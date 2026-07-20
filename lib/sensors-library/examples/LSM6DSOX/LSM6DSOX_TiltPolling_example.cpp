/*******************************************************************************
 * @file LSM6DSOX_ActivityDetection_example.cpp
 * @brief Example demonstrating wake-up and activity detection with LSM6DSOX.
 *
 * This example shows:
 * - Configuring wake-up/activity detection
 * - Using interrupts for motion detection
 * - Low power standby with wake on motion
 * - Threshold-based activity detection
 *
 * Note: The LSM6DSOX has embedded functions for activity/inactivity detection.
 * This example demonstrates the basic concept using data ready and thresholds.
 *
 * @version 0.1.0
 * @date 2025-11-19
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include <cmath>

#include "LSM6DSOX.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* I2C Port configuration */
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;
constexpr gpio_num_t kInt1Pin = GPIO_NUM_19;

// Create I2C instance
I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX Activity Detection Example >>\n\n");

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

    printf("LSM6DSOX initialized! Device ID: 0x%02X\n\n", imu.readDeviceID());

    // Configure for Tilt Detection
    printf("Configuring for Tilt Detection...\n");

    // 1. Set Accelerometer ODR to at least 26Hz (we use 26Hz)
    err = imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k26Hz);
    err |= imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);

    // Disable gyro to save power (Tilt only uses Accel)
    err |= imu.disableGyro();

    if (err != ESP_OK) {
        printf("Error configuring sensor settings\n");
    }

    // 2. Configure Tilt Logic
    err = imu.configureTiltDetection();
    if (err != ESP_OK) {
        printf("Error configuring Tilt Detection: %s\n", esp_err_to_name(err));
    }

    printf("\nConfiguration complete!\n");
    printf("Monitoring for TILT events...\n");
    printf("Tilt the device > 35 degrees to trigger.\n");
    printf("Note: 2-second settling time required after start.\n\n");

    uint32_t eventCount = 0;

    while (true) {
        // Poll the Tilt Status (Main Page)
        if (imu.checkTilt()) {
            eventCount++;
            printf("\n*** TILT DETECTED #%lu ***\n", eventCount);

            // Optional: Read accel data
            // We must call measure() to update the internal raw data buffers
            imu.measure();

            int16_t accel[3];
            imu.getAccelRawData(accel);
            printf("Accel: [%6d, %6d, %6d]\n", accel[0], accel[1], accel[2]);

            // Small delay to avoid spamming
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
