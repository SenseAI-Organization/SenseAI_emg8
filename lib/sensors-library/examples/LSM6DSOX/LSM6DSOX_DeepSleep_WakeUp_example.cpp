/*******************************************************************************
 * @file LSM6DSOX_DeepSleep_WakeUp_example.cpp
 * @brief Example of Ultra Low Power operation with Wake-Up from Deep Sleep & FIFO
 * History.
 *
 * This example demonstrates:
 * - Configuring the accelerometer in Ultra Low Power mode (12.5Hz).
 * - Configuring FIFO in Continuous mode to record history while sleeping.
 * - Configuring the Wake-Up (Activity) interrupt to detect motion.
 * - Putting the ESP32 into Deep Sleep.
 * - Waking up on motion, reading the FIFO history (pre-wake-up data).
 * - Processing the data: Discarding recent "shake" samples, averaging, and calculating
 * Roll.
 *
 * @version 0.2.0
 * @date 2025-11-27
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include <cmath>
#include <numeric>
#include <vector>

#include "LSM6DSOX.hpp"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;
constexpr gpio_num_t kINT1 = GPIO_NUM_19;  // Interrupt pin connected to INT1

I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);

// Helper struct for 3D vector
struct Vector3 {
    float x, y, z;
};

/**
 * @brief Calculates the average of a subset of vectors.
 * @param buffer The source vector of data.
 * @param startIdx The starting index in the buffer.
 * @param count The number of samples to average.
 * @return Vector3 The averaged vector.
 */
Vector3 calculateAverage(const std::vector<Vector3>& buffer, size_t startIdx,
                         size_t count) {
    Vector3 sum = {0, 0, 0};
    for (size_t i = 0; i < count; ++i) {
        sum.x += buffer[startIdx + i].x;
        sum.y += buffer[startIdx + i].y;
        sum.z += buffer[startIdx + i].z;
    }
    return {sum.x / count, sum.y / count, sum.z / count};
}

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX Deep Sleep Wake-Up & FIFO Example >>\n\n");

    LSM6DSOX imu(i2c1, LSM6DSOX::kAddressLow);

    ESP_ERROR_CHECK(i2c1.init());

    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();

    if (wakeupReason == ESP_SLEEP_WAKEUP_EXT0) {
        printf("Wake-up caused by Motion! Reading FIFO history...\n");

        // NOTE: We do NOT call imu.init() here because it performs a software reset
        // which would clear the FIFO data we want to read.
        // The sensor is already initialized and running from before the sleep.

        uint16_t samples = imu.getFifoStatus();
        printf("FIFO contains %d samples.\n", samples);

        if (samples > 0) {
            // 1. Read all data from FIFO
            std::vector<LSM6DSOX::FifoData> rawFifo(samples);
            imu.readFifoWords(rawFifo.data(), samples);

            // 2. Parse into Accelerometer vectors
            std::vector<Vector3> accelData;
            accelData.reserve(samples);

            int16_t raw[3];
            for (const auto& word : rawFifo) {
                if (imu.parseFifoAccelData(&word, raw)) {
                    // Convert to float (using raw LSBs is fine for atan2 ratio)
                    accelData.push_back({(float)raw[0], (float)raw[1], (float)raw[2]});
                }
            }

            printf("Parsed %d accelerometer samples.\n", accelData.size());

            // 3. Process Data
            // We want to discard the most recent samples (the "shake" that woke us up)
            const size_t kDiscardCount = 20;
            const size_t kWindowSize = 20;  // Rolling average window

            if (accelData.size() > kDiscardCount + kWindowSize) {
                size_t validSamples = accelData.size() - kDiscardCount;

                printf("Calculating Roll with rolling average (Window: %d)...\n",
                       kWindowSize);
                printf("--- Start Plot Data ---\n");  // Marker for plotter

                // Iterate through valid samples, applying rolling average
                // We start from index 0 (oldest) up to the point where we cut off
                for (size_t i = 0; i <= validSamples - kWindowSize; i++) {
                    Vector3 avg = calculateAverage(accelData, i, kWindowSize);

                    // Calculate Roll: atan2(Ay, sqrt(Ax^2 + Az^2))
                    // Result in degrees
                    float roll =
                        atan2(avg.y, sqrt(avg.x * avg.x + avg.z * avg.z)) * 57.29578f;

                    printf("Roll:%.2f\n", roll);
                }
                printf("--- End Plot Data ---\n");
            } else {
                printf("Not enough samples for processing (Need > %d).\n",
                       kDiscardCount + kWindowSize);
            }
        } else {
            printf("FIFO Empty! (Did the sensor reset?)\n");
        }

        // Now we can reset the sensor for a clean state for the next cycle
        imu.softwareReset();
        vTaskDelay(pdMS_TO_TICKS(20));

    } else {
        printf("Normal startup (Power On or Reset)\n");

        // Initialize sensor (performs soft reset)
        esp_err_t err = imu.init();
        if (err != ESP_OK) {
            printf("Error initializing LSM6DSOX: %s\n", esp_err_to_name(err));
            return;
        }
    }

    printf("Re-configuring for Deep Sleep...\n");

    // 1. Configure Accelerometer for Ultra Low Power
    // Set ODR to 12.5Hz
    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k12_5Hz);
    imu.configureAccelPowerMode(LSM6DSOX::AccelPowerMode::kUltraLowPower);
    imu.disableGyro();

    // 2. Configure FIFO
    // Continuous Mode: Overwrites oldest data when full
    // Batch Data Rate: 12.5Hz for Accel
    // Watermark: 0 (We don't use FIFO interrupt, we use Activity interrupt)
    imu.configureFIFO(LSM6DSOX::FifoMode::kContinuous, 0,
                      LSM6DSOX::BatchDataRate::k12_5Hz,
                      LSM6DSOX::BatchDataRate::kNotBatched);

    // 3. Configure Wake-Up (Activity) Interrupt
    // Threshold: ~62mg, Duration: 0 (Immediate)
    imu.configureWakeUp(2, 0);
    imu.enableWakeUpInterrupt(true, LSM6DSOX::Interrupt::kInt1);
    imu.configurePin(kINT1, LSM6DSOX::Interrupt::kInt1, true, false);

    // 4. Configure ESP32 for Deep Sleep
    rtc_gpio_isolate(GPIO_NUM_12);
    esp_sleep_enable_ext0_wakeup(kINT1, 1);

    printf("Going to sleep. FIFO is recording. Shake to wake and view history!\n");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Allow UART to flush
    esp_deep_sleep_start();
}
