/*******************************************************************************
 * @file LSM6DSOX_Orientation_example.cpp
 * @brief Orientation example for LSM6DSOX IMU sensor using built-in methods.
 *
 * This example demonstrates:
 * - Basic sensor initialization
 * - Reading accelerometer and gyroscope data
 * - Calculating roll and pitch angles using class methods
 *
 * @version 0.1.0
 * @date 2025-11-26
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "LSM6DSOX.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;
I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX Orientation Example >>\n\n");

    // Create sensor instance (SDO/SA0 to GND = 0x6A, to VDD = 0x6B)
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
        printf("Check connections and I2C address!\n");
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    printf("LSM6DSOX found! Device ID: 0x%02X\n", imu.readDeviceID());

    // Configure accelerometer: ±2g range, 104Hz ODR
    err = imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    if (err != ESP_OK) {
        printf("Error configuring accelerometer scale\n");
    }

    err = imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k104Hz);
    if (err != ESP_OK) {
        printf("Error configuring accelerometer data rate\n");
    }

    // Configure gyroscope: ±250dps range, 104Hz ODR
    err = imu.configureGyroFullScale(LSM6DSOX::GyroFullScale::k250dps);
    if (err != ESP_OK) {
        printf("Error configuring gyroscope scale\n");
    }

    err = imu.configureGyroDataRate(LSM6DSOX::GyroOutputDataRate::k104Hz);
    if (err != ESP_OK) {
        printf("Error configuring gyroscope data rate\n");
    }

    printf("Configuration complete!\n");
    printf("Starting measurements...\n\n");

    vTaskDelay(pdMS_TO_TICKS(100));

    while (true) {
        // Wait for data to be ready
        while (!imu.checkAccelDataReady() || !imu.checkGyroDataReady()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Read all sensor data
        err = imu.measure();
        if (err != ESP_OK) {
            printf("Error reading sensor data: %s\n", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Calculate angles using class methods
        float roll = imu.getRoll();
        float pitch = imu.getPitch();

        // Print data
        printf("Angles: Roll=%6.2f deg, Pitch=%6.2f deg\n", roll, pitch);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
