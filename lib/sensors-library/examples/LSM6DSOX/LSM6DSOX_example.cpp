/*******************************************************************************
 * @file LSM6DSOX_example.cpp
 * @brief Basic example for LSM6DSOX IMU sensor with I2C interface.
 *
 * This example demonstrates:
 * - Basic sensor initialization
 * - Reading accelerometer and gyroscope data
 * - Reading temperature
 * - Calculating roll and pitch angles from accelerometer
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

// I2C
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;
I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);

// Sensitivity values for converting raw data to physical units
// These depend on the full-scale range configured
constexpr float kAccelSensitivity_2g = 0.061f;    // mg/LSB for ±2g range
constexpr float kGyroSensitivity_250dps = 8.75f;  // mdps/LSB for ±250dps range

/**
 * @brief Calculate roll angle from accelerometer data
 * @param ax Acceleration in X-axis (mg)
 * @param ay Acceleration in Y-axis (mg)
 * @param az Acceleration in Z-axis (mg)
 * @return Roll angle in degrees
 */
float calculateRoll(float ax, float ay, float az) {
    return atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;
}

/**
 * @brief Calculate pitch angle from accelerometer data
 * @param ax Acceleration in X-axis (mg)
 * @param ay Acceleration in Y-axis (mg)
 * @param az Acceleration in Z-axis (mg)
 * @return Pitch angle in degrees
 */
float calculatePitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
}

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX Basic Example >>\n\n");

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

        // Read temperature
        imu.readTemperature();

        // Get raw data
        int16_t accelRaw[3], gyroRaw[3];
        imu.getAccelRawData(accelRaw);
        imu.getGyroRawData(gyroRaw);

        // Convert to physical units
        float ax = accelRaw[0] * kAccelSensitivity_2g;  // mg
        float ay = accelRaw[1] * kAccelSensitivity_2g;  // mg
        float az = accelRaw[2] * kAccelSensitivity_2g;  // mg

        float gx = gyroRaw[0] * kGyroSensitivity_250dps;  // mdps
        float gy = gyroRaw[1] * kGyroSensitivity_250dps;  // mdps
        float gz = gyroRaw[2] * kGyroSensitivity_250dps;  // mdps

        // Calculate angles
        float roll = calculateRoll(ax, ay, az);
        float pitch = calculatePitch(ax, ay, az);

        // Print data
        printf("Ax: %7.2f, Ay: %7.2f, Az: %7.2f, ", ax, ay, az);
        printf("Gx: %7.2f, Gy: %7.2f, Gz: %7.2f, ", gx / 1000.0f, gy / 1000.0f,
               gz / 1000.0f);
        printf("Angles: Roll=%6.2f deg, Pitch=%6.2f deg\n", roll, pitch);
        printf("Temp: %.2f°C\n", imu.getTemperatureCelsius());

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
