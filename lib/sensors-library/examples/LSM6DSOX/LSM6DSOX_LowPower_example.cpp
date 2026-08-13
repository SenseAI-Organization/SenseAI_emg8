/*******************************************************************************
 * @file LSM6DSOX_LowPower_example.cpp
 * @brief Example demonstrating low power modes of the LSM6DSOX sensor.
 *
 * This example shows:
 * - Configuring different ODR (Output Data Rates) for power savings
 * - Using lower full-scale ranges
 * - Enabling/disabling gyroscope to save power
 * - Power-down mode
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

// Create I2C instance
I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);

/**
 * @brief Demonstrate high performance mode
 */
void highPerformanceMode(LSM6DSOX& imu) {
    printf("\n=== HIGH PERFORMANCE MODE ===\n");
    printf("Accel: ±16g @ 1.67kHz, Gyro: ±2000dps @ 1.67kHz\n");

    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k16g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k1667Hz);
    imu.configureGyroFullScale(LSM6DSOX::GyroFullScale::k2000dps);
    imu.configureGyroDataRate(LSM6DSOX::GyroOutputDataRate::k1667Hz);

    printf("Highest current consumption, best performance\n");
}

/**
 * @brief Demonstrate normal mode
 */
void normalMode(LSM6DSOX& imu) {
    printf("\n=== NORMAL MODE ===\n");
    printf("Accel: ±4g @ 104Hz, Gyro: ±500dps @ 104Hz\n");

    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k4g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k104Hz);
    imu.configureGyroFullScale(LSM6DSOX::GyroFullScale::k500dps);
    imu.configureGyroDataRate(LSM6DSOX::GyroOutputDataRate::k104Hz);

    printf("Balanced power and performance\n");
}

/**
 * @brief Demonstrate low power mode
 */
void lowPowerMode(LSM6DSOX& imu) {
    printf("\n=== LOW POWER MODE ===\n");
    printf("Accel: ±2g @ 12.5Hz, Gyro: OFF\n");

    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k12_5Hz);
    imu.disableGyro();  // Turn off gyroscope to save power

    printf("Very low current consumption\n");
}

/**
 * @brief Demonstrate ultra low power mode
 */
void ultraLowPowerMode(LSM6DSOX& imu) {
    printf("\n=== ULTRA LOW POWER MODE ===\n");
    printf("Both sensors in power-down mode\n");

    imu.disableAccel();
    imu.disableGyro();

    printf("Minimal current consumption (only I2C interface active)\n");
}

/**
 * @brief Demonstrate accelerometer-only mode
 */
void accelOnlyMode(LSM6DSOX& imu) {
    printf("\n=== ACCELEROMETER ONLY MODE ===\n");
    printf("Accel: ±2g @ 26Hz, Gyro: OFF\n");

    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k26Hz);
    imu.disableGyro();

    printf("Good for motion detection, orientation sensing\n");
}

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX Low Power Modes Example >>\n\n");

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
    printf("\nThis example demonstrates different power modes.\n");
    printf("Each mode will run for 5 seconds.\n");

    while (true) {
        // Cycle through different power modes

        // Mode 1: High Performance
        highPerformanceMode(imu);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Read some samples in this mode
        printf("Reading 10 samples...\n");
        for (int i = 0; i < 10; i++) {
            while (!imu.checkAccelDataReady() || !imu.checkGyroDataReady()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            imu.measure();
            int16_t accel[3], gyro[3];
            imu.getAccelRawData(accel);
            imu.getGyroRawData(gyro);
            printf("  A:[%6d,%6d,%6d] G:[%6d,%6d,%6d]\n", accel[0], accel[1], accel[2],
                   gyro[0], gyro[1], gyro[2]);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Mode 2: Normal Mode
        normalMode(imu);
        vTaskDelay(pdMS_TO_TICKS(5000));

        printf("Reading 10 samples...\n");
        for (int i = 0; i < 10; i++) {
            while (!imu.checkAccelDataReady() || !imu.checkGyroDataReady()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            imu.measure();
            int16_t accel[3], gyro[3];
            imu.getAccelRawData(accel);
            imu.getGyroRawData(gyro);
            printf("  A:[%6d,%6d,%6d] G:[%6d,%6d,%6d]\n", accel[0], accel[1], accel[2],
                   gyro[0], gyro[1], gyro[2]);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Mode 3: Accelerometer Only
        accelOnlyMode(imu);
        vTaskDelay(pdMS_TO_TICKS(5000));

        printf("Reading 10 samples (accel only)...\n");
        for (int i = 0; i < 10; i++) {
            while (!imu.checkAccelDataReady()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            imu.measure();
            int16_t accel[3];
            imu.getAccelRawData(accel);
            printf("  A:[%6d,%6d,%6d]\n", accel[0], accel[1], accel[2]);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Mode 4: Low Power
        lowPowerMode(imu);
        vTaskDelay(pdMS_TO_TICKS(5000));

        printf("Reading 10 samples (low rate)...\n");
        for (int i = 0; i < 10; i++) {
            while (!imu.checkAccelDataReady()) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            imu.measure();
            int16_t accel[3];
            imu.getAccelRawData(accel);
            printf("  A:[%6d,%6d,%6d]\n", accel[0], accel[1], accel[2]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Mode 5: Ultra Low Power (both sensors off)
        ultraLowPowerMode(imu);
        printf("Sleeping for 5 seconds with sensors off...\n");
        vTaskDelay(pdMS_TO_TICKS(5000));

        printf("\n--- Cycling back to start ---\n\n");
    }
}
