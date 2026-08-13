/*******************************************************************************
 * @file LSM6DSOX_DeepSleep_example.cpp
 * @brief Example combining FIFO with deep sleep for power optimization.
 *
 * This example demonstrates:
 * - Using FIFO to buffer data during deep sleep (Continuous Mode)
 * - Waking up on Timer (periodically)
 * - Reading batched data after wake-up
 * - Reconstructing the timeline of the data
 *
 * @version 0.2.0
 * @date 2025-11-24
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "LSM6DSOX.hpp"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

/* I2C Port configuration */
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;

// FIFO configuration
// We use a watermark just to demonstrate configuration, though we wake on timer.
constexpr uint16_t kFifoWatermark = 100;
constexpr uint64_t kDeepSleepDurationUs = 5 * 1000 * 1000;  // 5 seconds

// RTC data persists across deep sleep
RTC_DATA_ATTR bool sensorConfigured = false;
RTC_DATA_ATTR uint32_t wakeupCount = 0;

/**
 * @brief Initialize and configure the LSM6DSOX sensor
 */
esp_err_t configureSensor(LSM6DSOX& imu) {
    esp_err_t err = imu.init();
    if (err != ESP_OK) {
        return err;
    }

    // Reset to default to clear any previous state
    err |= imu.softwareReset();
    vTaskDelay(pdMS_TO_TICKS(20));

    // Configure for low power but enough to fill FIFO during sleep
    // 26Hz ODR. 5s sleep -> ~130 samples per channel.
    // Total words ~ 260. Fits in 3KB FIFO.
    err |= imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    err |= imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k26Hz);
    err |= imu.configureGyroFullScale(LSM6DSOX::GyroFullScale::k250dps);
    err |= imu.configureGyroDataRate(LSM6DSOX::GyroOutputDataRate::k26Hz);

    // Configure FIFO in Continuous mode (overwrites old data if full)
    // We set BDR (Batch Data Rate) equal to ODR
    err |=
        imu.configureFIFO(LSM6DSOX::FifoMode::kContinuous, kFifoWatermark,
                          LSM6DSOX::BatchDataRate::k26Hz, LSM6DSOX::BatchDataRate::k26Hz);

    return err;
}

/**
 * @brief Process FIFO data after wakeup
 */
void processFifoData(LSM6DSOX& imu) {
    uint16_t fifoLevel = imu.getFifoStatus();
    printf("FIFO Level: %d words\n", fifoLevel);

    LSM6DSOX::FifoData data;
    int16_t accel[3], gyro[3];

    // ODR = 26Hz -> Period = ~38.46ms
    const float kPeriodMs = 1000.0f / 26.0f;

    printf("Reconstructing data history (ODR=26Hz, Period=%.2fms)...\n", kPeriodMs);
    printf("Index | Type  |    X   |    Y   |    Z   | Est. Time (ms ago)\n");
    printf("------+-------+--------+--------+--------+-------------------\n");

    // Change this to get a different number of samples from FIFO
    for (int i = 0; i < fifoLevel; i++) {
        if (imu.readFifoWord(&data) == ESP_OK) {
            // Calculate how many samples "ago" this is.
            // We are reading Oldest to Newest.
            // Assuming 2 words (Accel + Gyro) per time step.
            float timeAgo = ((fifoLevel - 1 - i) / 2.0f) * kPeriodMs;

            if (imu.parseFifoAccelData(&data, accel)) {
                printf("%5d | ACCEL | %6d | %6d | %6d | -%.2f ms\n", i, accel[0],
                       accel[1], accel[2], timeAgo);
            } else if (imu.parseFifoGyroData(&data, gyro)) {
                printf("%5d | GYRO  | %6d | %6d | %6d | -%.2f ms\n", i, gyro[0], gyro[1],
                       gyro[2], timeAgo);
            }
        }
    }
}

extern "C" void app_main() {
    wakeupCount++;

    // Initialize I2C
    I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);
    esp_err_t err = i2c1.init();
    if (err != ESP_OK) {
        printf("Error initializing I2C: %s\n", esp_err_to_name(err));
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    // Create sensor instance
    LSM6DSOX imu(i2c1, LSM6DSOX::kAddressLow);

    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();

    if (wakeupReason == ESP_SLEEP_WAKEUP_TIMER) {
        printf("\n=== Wakeup #%lu from Timer ===\n", wakeupCount);
        // Process the data collected during sleep
        processFifoData(imu);
    } else {
        printf("\n\n<< LSM6DSOX Deep Sleep FIFO Example >>\n");
        printf("First boot / Reset\n");

        printf("Configuring sensor...\n");
        if (configureSensor(imu) == ESP_OK) {
            printf("Sensor configured. ODR = 26Hz.\n");
            sensorConfigured = true;
        } else {
            printf("Sensor configuration failed!\n");
        }
    }

    printf("Entering deep sleep for %llu seconds...\n", kDeepSleepDurationUs / 1000000);

    // Configure Timer Wakeup
    esp_sleep_enable_timer_wakeup(kDeepSleepDurationUs);

    // Small delay to allow UART to finish
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enter deep sleep
    esp_deep_sleep_start();
}
