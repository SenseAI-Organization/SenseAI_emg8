/*******************************************************************************
 * @file LSM6DSOX_ActivityDetection_example.cpp
 * @brief Example demonstrating wake-up and activity detection with LSM6DSOX.
 *
 * This example shows:
 * - Configuring tilt wake-up/activity detection
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

// Interrupt flag
volatile bool tiltInterruptFired = false;

// ISR Handler
static void IRAM_ATTR tiltIsrHandler(void* arg) {
    tiltInterruptFired = true;
}

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

    // 1. Set Accelerometer ODR to at least 26Hz
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

    // Enable Tilt Interrupt on INT1
    err = imu.enableTiltInterrupt(true, LSM6DSOX::Interrupt::INT1);
    if (err != ESP_OK) {
        printf("Error enabling Tilt Interrupt: %s\n", esp_err_to_name(err));
    }

    // 3. Configure MCU GPIO for Interrupts
    printf("Configuring GPIO %d for interrupts...\n", kInt1Pin);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;  // Trigger on rising edge
    io_conf.pin_bit_mask = (1ULL << kInt1Pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;  // Assume active high
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Install ISR service
    gpio_install_isr_service(0);
    // Hook ISR handler
    gpio_isr_handler_add(kInt1Pin, tiltIsrHandler, nullptr);

    printf("\nConfiguration complete!\n");
    printf("Monitoring for TILT events (Interrupt Driven)...\n");
    printf("Tilt the device > 35 degrees to trigger.\n");
    printf("Note: 2-second settling time required after start.\n\n");

    uint32_t eventCount = 0;

    while (true) {
        // Check if interrupt fired
        if (tiltInterruptFired) {
            tiltInterruptFired = false;  // Reset flag

            // Check the source register to confirm it was Tilt and clear the latch
            if (imu.checkTilt()) {
                eventCount++;
                printf("\n*** TILT INTERRUPT DETECTED #%lu ***\n", eventCount);

                // Read accel data
                imu.measure();
                int16_t accel[3];
                imu.getAccelRawData(accel);
                printf("Accel: [%6d, %6d, %6d]\n", accel[0], accel[1], accel[2]);
            } else {
                printf("Interrupt fired but Tilt flag not set?\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Short delay to yield
    }
}
