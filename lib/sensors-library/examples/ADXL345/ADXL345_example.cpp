/*******************************************************************************
 * @file ADXL345_example.cpp
 * @brief File to test the ADXL345 class.
 *
 * @version 0.1.3
 * @date 2024-12-10
 * @author Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "ADXL345.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* I2C Port Brain v0.2 --> SDA GPIO_NUM_4, SCL GPIO_NUM_5
   Serial port Brain v0.2 --> SDA GPIO_NUM_17, SCL GPIO_NUM_18 */
constexpr gpio_num_t kSDA1 = GPIO_NUM_5;
constexpr gpio_num_t kSCL1 = GPIO_NUM_4;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);  // Change I2C pins

// constexpr gpio_num_t kSDA0 = GPIO_NUM_17;
// constexpr gpio_num_t kSCL0 = GPIO_NUM_18;
// I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 100000, true); // Change I2C pins

extern "C" void app_main() {
    printf("\n\n<< ADXL345 Example has started! >>\n\n");

    ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);  // SDO to GND

    esp_err_t err = i2c1.init();
    if (err) {
        printf("Error initializing I2C1: %s\n", esp_err_to_name(err));
        while (1);
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
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (true) {
        // Accel connected to the i2c port
        while (!accel_i2c.checkDataReady());  // Wait for data to be ready (polling)

        err = accel_i2c.measure();
        if (err) {
            printf("Error measuring the ADXL345: %d\n", err);
        }

        int16_t data[3] = {0};
        accel_i2c.getRawData(data);
        printf("%i, %i, %i\n", data[0], data[1], data[2]);

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
