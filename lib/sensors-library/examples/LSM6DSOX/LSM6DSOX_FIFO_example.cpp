/*******************************************************************************
 * @file LSM6DSOX_FIFO_example.cpp
 * @brief Example demonstrating FIFO usage with the LSM6DSOX sensor.
 *
 * This example shows:
 * - FIFO configuration in continuous mode
 * - Using watermark interrupt to know when FIFO has enough samples
 * - Reading batched data from FIFO
 *
 * @version 0.1.0
 * @date 2025-11-19
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "LSM6DSOX.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* I2C Port configuration */
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;
constexpr gpio_num_t kInt1Pin = GPIO_NUM_19;

// FIFO configuration
constexpr uint16_t kFifoWatermark = 50;  // Generate interrupt when 50 samples available

// Create I2C instance
I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX FIFO Example >>\n\n");

    LSM6DSOX imu(i2c1, LSM6DSOX::kAddressLow);

    // Initialize I2C
    esp_err_t err = i2c1.init();
    if (err != ESP_OK) {
        printf("Error initializing I2C: %s\n", esp_err_to_name(err));
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    // Initialize sensor
    err = imu.init();
    if (err != ESP_OK) {
        printf("Error initializing LSM6DSOX: %s\n", esp_err_to_name(err));
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    printf("LSM6DSOX initialized! Device ID: 0x%02X\n", imu.readDeviceID());

    // 1. Configure FIFO FIRST (sets BDR, Watermark, Mode)
    // This must be done before enabling sensors (ODR)
    // We set BDR to 104Hz for both Accel and Gyro to match ODR
    err = imu.configureFIFO(LSM6DSOX::FifoMode::kContinuous, kFifoWatermark,
                            LSM6DSOX::BatchDataRate::k104Hz,
                            LSM6DSOX::BatchDataRate::k104Hz);
    if (err != ESP_OK) {
        printf("Error configuring FIFO: %s\n", esp_err_to_name(err));
    } else {
        printf("FIFO configured successfully\n");
    }

    // 2. Configure accelerometer and gyroscope (Enable Sensors)
    // ODR must be >= BDR (we set BDR to 104Hz in configureFIFO)
    err = imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    err |= imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k104Hz);
    err |= imu.configureGyroFullScale(LSM6DSOX::GyroFullScale::k250dps);
    err |= imu.configureGyroDataRate(LSM6DSOX::GyroOutputDataRate::k104Hz);

    if (err != ESP_OK) {
        printf("Error configuring sensor ODR/Scale\n");
    }

    // Give sensors time to start producing data
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check initial FIFO status
    uint16_t initialFifoLevel = imu.getFifoStatus();
    printf("Initial FIFO Level: %d samples\n", initialFifoLevel);

    printf("FIFO configuration complete!\n");
    printf("Collecting FIFO data (watermark = %d samples)...\n\n", kFifoWatermark);

    uint32_t batchCount = 0;

    while (true) {
        // Poll FIFO status instead of using interrupts
        uint16_t fifoLevel = imu.getFifoStatus();

        // Wait until FIFO reaches watermark level
        if (fifoLevel >= kFifoWatermark) {
            batchCount++;

            printf("=== Batch #%lu ===\n", batchCount);
            printf("FIFO Level: %d words (samples)\n", fifoLevel);

            // Read FIFO data using the proper FIFO reading procedure
            // Each FIFO word = 1 tag byte + 6 data bytes
            LSM6DSOX::FifoData fifoWords[kFifoWatermark];

            err = imu.readFifoWords(fifoWords, kFifoWatermark);
            if (err != ESP_OK) {
                printf("Error reading FIFO: %s\n", esp_err_to_name(err));
            } else {
                // Parse and display the FIFO data
                int16_t accel[3], gyro[3];
                uint16_t accelCount = 0, gyroCount = 0;

                for (uint16_t i = 0; i < kFifoWatermark; i++) {
                    // Check tag and parse accordingly
                    if (imu.parseFifoAccelData(&fifoWords[i], accel)) {
                        if (accelCount < 5) {  // Print first 5 accelerometer samples
                            printf("  Accel[%2d]: [%6d, %6d, %6d]\n", accelCount,
                                   accel[0], accel[1], accel[2]);
                        }
                        accelCount++;
                    } else if (imu.parseFifoGyroData(&fifoWords[i], gyro)) {
                        if (gyroCount < 5) {  // Print first 5 gyroscope samples
                            printf("  Gyro[%2d]:  [%6d, %6d, %6d]\n", gyroCount, gyro[0],
                                   gyro[1], gyro[2]);
                        }
                        gyroCount++;
                    }
                }

                printf("Total: %d accel samples, %d gyro samples\n", accelCount,
                       gyroCount);
            }

            printf("\n");
        } else {
            // Print status every second
            static uint32_t s_lastPrint = 0;
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - s_lastPrint > 1000) {
                printf("FIFO Level: %d / %d samples\n", fifoLevel, kFifoWatermark);
                s_lastPrint = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
