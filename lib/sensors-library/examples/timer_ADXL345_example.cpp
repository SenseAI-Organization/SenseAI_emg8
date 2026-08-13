/*******************************************************************************
 * @file timer_ADXL345_example.cpp
 * @brief An example code that shows how to sample accel readings using timer
 * interrupts.
 *
 * @version v0.1.0
 * @date 2025-03-11
 * @author emmanuel@sense-ai.co, Sense-AI
 * 
 * @note The Terminal baudrate should be configured with a value of 921600
 *******************************************************************************
 *******************************************************************************/

#include "ADXL345.hpp"
#include "smart_sensor_sense.hpp"
#include "driver/uart.h"

constexpr uint8_t kSet   = 1;
constexpr uint8_t kClear = 0;

volatile uint8_t measureFlag = kClear;

/**
 * @brief Timer callback function.
 *
 * This callback toggles an LED every time the timer alarm fires.
 */
void IRAM_ATTR timerCallback(void* arg) {
    measureFlag = kSet;
}

extern "C" void app_main() {
    uart_set_baudrate(UART_NUM_0, 921600);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n\n<< Timer & ADXL345 Example has started! >>\n\n");

    const gpio_num_t kSDA1 = GPIO_NUM_5;
    const gpio_num_t kSCL1 = GPIO_NUM_4;
    I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);   // Change I2C pins

    ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);   // SDO to GND

    esp_err_t err = i2c1.init();
    if (err) {
        printf("Error initializing I2C1: %s\n", esp_err_to_name(err));
        while(1);
    }

    err = accel_i2c.init();
    if (err) {
        printf("Error initializing ADXL345: %s. Trying again. Status: ",
               esp_err_to_name(err));
        err = accel_i2c.init();
        printf("%s\n", esp_err_to_name(err));
    }

    printf("Calibrating the ADXL345 sensor (i2c). Don't move it!\n");
    err = accel_i2c.calibrate();
    if (err) {
        printf("Error calibrating ADXL345: %s.\n", esp_err_to_name(err));
        while(1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    err = accel_i2c.disableMeasure();   // Disable measures before config
    err |= accel_i2c.configureOutPutDataRate(ADXL345::OutputDataRate::k800Hz);
    err |= accel_i2c.enableMeasure();
    if (err) {
        printf("Error with ADXL345 configuration: %s.\n", esp_err_to_name(err));
        while(1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    Timer timer(3);

    // Initialize the timer hardware.
    if (timer.init() != ESP_OK) {
        printf("Failed to initialize timer.\n");
        return;
    }

    // Configure the timer alarm with the current settings.
    if (timer.configure() != ESP_OK) {
        printf("Failed to configure timer.\n");
        return;
    }

    // Set the callback function to be executed on each alarm.
    timer.setCallback(timerCallback);

    timer.enableInterrupts();

    // Start the timer counter.
    if (timer.start() != ESP_OK) {
        printf("Failed to start timer.\n");
        return;
    }

    const uint16_t kDataSize = 32;
    int16_t accelZ[kDataSize] = {0};
    uint16_t sample = 0;
    while (true) {
        if (measureFlag) {
            measureFlag = kClear;

            err = accel_i2c.measure();
            if (err) {
                printf("Error measuring the ADXL345: %d\n", err);
            }

            int16_t data[3] = {0};
            accel_i2c.getRawData(data);

            accelZ[sample] = data[2]; // Z data
            ++sample;
        }

        if (sample >= kDataSize) {
            // printf("\nStart\n");
            for (uint16_t i = 0; i < kDataSize; i++) {
                printf("%i\n", accelZ[i]);
            }
            // printf("\nEnd\n");
            sample = 0;
        }
    }
}