/*******************************************************************************
 * @file ADXL345_ACT_INT_example.cpp
 * @brief File to test the ADXL345 class.
 * 
 * @version 0.1.1
 * @date 2024-3-11
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "ADXL345.hpp"

/* I2C Port Brain v0.2 --> SDA GPIO_NUM_4, SCL GPIO_NUM_5
   Serial port Brain v0.2 --> SDA GPIO_NUM_17, SCL GPIO_NUM_18 */
constexpr gpio_num_t kSDA1 = GPIO_NUM_4;
constexpr gpio_num_t kSCL1 = GPIO_NUM_5;
constexpr gpio_num_t kInt1Pin = GPIO_NUM_19;
constexpr gpio_num_t kInt2Pin = GPIO_NUM_20;

volatile bool activityFlag = false;

I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false); // Change I2C pins

ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);   // SDO to GND

void onActivity(void* context) {
    activityFlag = true;
    accel_i2c.disablePinInterrupt(ADXL345::Interrupt::INT2);
}

extern "C" void app_main() {
    printf("\n\n<< ADXL345 Example has started! >>\n\n");

    // I2C Instance initialization
    esp_err_t err = i2c1.init();
    if (err != ESP_OK) {
        printf("Error initializing I2C1: %s\n", esp_err_to_name(err));
        while(1);
    }

    // Sensor initialization
    err = accel_i2c.init();
    if (err != ESP_OK) {
        printf("Error initializing ADXL345: %s. Trying again. Status: ",
               esp_err_to_name(err));
        err = accel_i2c.init();
        printf("%s\n", esp_err_to_name(err));
    }

    // Configure the sampling frequency.
    err = accel_i2c.configureOutPutDataRate(ADXL345::OutputDataRate::k100Hz);
    if (err != ESP_OK) {
        printf("Error on output data rate configuration\r\n");
        return;
    }

    printf("Calibrating the ADXL345 sensor (i2c). Don't move it!\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Axis offset calibration
    err = accel_i2c.calibrate();
    if (err != ESP_OK) {
        printf("Error calibrating ADXL345: %s.\n", esp_err_to_name(err));
        while(1);
    }

    printf("Calibration completed!\n");

    err = accel_i2c.disableMeasure(); // Disable measures before configuration
    if (err != ESP_OK) {
        printf("Error disabling the measures: %s.\n", esp_err_to_name(err));
        return;
    }

    err = accel_i2c.configureOutPutDataRate(ADXL345::OutputDataRate::k800Hz);
    if (err != ESP_OK) {
        printf("Error on output data rate configuration\r\n");
        return;
    }

    const uint8_t kThreshold_03G = 0x05;      // 0.3 g
    err = accel_i2c.configureActivityThreshold(kThreshold_03G);
    err |= accel_i2c.configureActivityControl(
        ADXL345::ActivityControl::kActZ |     // Activity on Z axis
        ADXL345::ActivityControl::kActAcDc
    );
    if (err != ESP_OK) {
        printf("Error on activity detection configuration\r\n");
        return;
    }

    // Assign Activity interrupt --> The flag is cleared by reading int_source_reg
    err = accel_i2c.assignInterrupts(ADXL345::InterruptFlags::kActivity,
                                     ADXL345::Interrupt::INT2);
    if (err != ESP_OK) {
        printf("Failed to assign interrupts.\n");
        return;
    }

    // Configure GPIO pin for INT2, rising edge
    err = accel_i2c.configurePin(kInt2Pin, ADXL345::Interrupt::INT2,
                                 ActiveLevel::kHigh);
    if (err != ESP_OK) {
        printf("Failed to configure INT2 GPIO pin.\n");
        return;
    }

    // Attach the interrupt (INT2) callback
    err = accel_i2c.attachInterrupt(ADXL345::Interrupt::INT2, onActivity,
                                    &accel_i2c);
    if (err != ESP_OK) {
        printf("Failed to attach interrupt callback.\n");
        printf("%s\n", esp_err_to_name(err));
        return;
    }

    // Enable the interrupt generation from the sensor
    err = accel_i2c.enableInterrupts(ADXL345::InterruptFlags::kActivity);
    if (err != ESP_OK) {
        printf("Failed to enable interrupt from the sensor.\n");
        return;
    }

    // Enable the interrupt from the mcu
    err = accel_i2c.enablePinInterrupt(ADXL345::Interrupt::INT2);
    if (err != ESP_OK) {
        printf("Failed to enable the pin interrupt.\n");
        return;
    }

    printf("Accel Configuration finished!\n");
    accel_i2c.enableMeasure();

    while (true) {
        if (activityFlag) {
            activityFlag = false;      // Clear the interrupt flag

            vTaskDelay(pdMS_TO_TICKS(100)); // Prevent multiple interrupts

            // Clear the accel activity flag by reading the int source reg
            if (accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity)) {
                printf("ACTIVITY DETECTED!\n");
            }

            accel_i2c.enablePinInterrupt(ADXL345::Interrupt::INT2);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}