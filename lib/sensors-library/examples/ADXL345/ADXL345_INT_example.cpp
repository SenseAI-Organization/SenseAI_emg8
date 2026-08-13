/*******************************************************************************
 * @file ADXL345_example.cpp
 * @brief File to test the ADXL345 class.
 * 
 * @version 0.1.4
 * @date 2024-03-11
 * @author Sense AI
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

volatile bool dataReady = false;

I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false); // Change I2C pins

ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);   // SDO to GND

void int2Callback(void* context) {
    dataReady = true;
}

extern "C" void app_main() {
    printf("\n\n<< ADXL345 Example has started! >>\n\n");

    // I2C Instance initialization
    esp_err_t err = i2c1.init();
    if (err) {
        printf("Error initializing I2C1: %s\n", esp_err_to_name(err));
        while(1);

    }
    // Sensor initialization
    err = accel_i2c.init();
    if (err) {
        printf("Error initializing ADXL345: %s. Trying again. Status: ",
               esp_err_to_name(err));
        err = accel_i2c.init();
        printf("%s\n", esp_err_to_name(err));
    }

    // Configure the sampling frequency. It must be equal or less than 100 Hz
    err = accel_i2c.configureOutPutDataRate(ADXL345::OutputDataRate::k25Hz);
    if (err) {
        printf("Error on output data rate configuration\r\n");
        return;
    }

    printf("Calibrating the ADXL345 sensor (i2c). Don't move it!\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Axis offset calibration
    err = accel_i2c.calibrate();
    if (err) {
        printf("Error calibrating ADXL345: %s.\n", esp_err_to_name(err));
        while(1);
    }

    printf("Calibration completed!\n");

    // Assign data ready interrupt --> The flag is cleared by reading the data
    err = accel_i2c.assignInterrupts(ADXL345::InterruptFlags::kDataReady,
                                     ADXL345::Interrupt::INT2);
    if (err != ESP_OK) {
        printf("Failed to assign interrupts.\n");
        return;
    }

    // Configure GPIO pin for INT1, rising edge
    err = accel_i2c.configurePin(kInt2Pin, ADXL345::Interrupt::INT2,
                                 ActiveLevel::kHigh);
    if (err != ESP_OK) {
        printf("Failed to configure INT1 GPIO pin.\n");
        return;
    }

    // Attach the interrupt (INT1) callback
    err = accel_i2c.attachInterrupt(ADXL345::Interrupt::INT2, int2Callback,
                                    &accel_i2c);
    if (err != ESP_OK) {
        printf("Failed to attach interrupt callback.\n");
        printf("%s\n", esp_err_to_name(err));
        return;
    }

    // Enable the interrupt from the mcu
    err = accel_i2c.enablePinInterrupt(ADXL345::Interrupt::INT2);
    if (err != ESP_OK) {
        printf("Failed to enable the pin interrupt.\n");
        return;
    }

    // Enable the interrupt generation from the sensor
    err = accel_i2c.enableInterrupts(ADXL345::InterruptFlags::kDataReady);
    if (err != ESP_OK) {
        printf("Failed to enable interrupt from the sensor.\n");
        return;
    }

    while (true) {
        if (dataReady) {
            dataReady = false;      // Clear the interrupt flag
            accel_i2c.measure();    // Clear the accel data ready flag

            int16_t data[3] = {0};
            accel_i2c.getRawData(data);
            printf("%i, %i, %i\n\r", data[0], data[1], data[2]);
        }

        asm volatile ("nop");       // Avoid busy looping
    }
}