/*******************************************************************************
 * @file rgb_led_example.cpp
 * @brief File to test the RGB LED class.
 *
 * @version 0.1.0
 * @date 2024-08-16
 * @author Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "actuators_sense.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// Define the GPIO pin for the RGB LED (GPIO 48 is standard for ESP32-S3-DevKitC-1)
#define RGB_LED_GPIO GPIO_NUM_48

extern "C" void app_main() {
    // Instantiate the RGB object with the specific GPIO pin and initial color (White)
    RGB brainLED(RGB_LED_GPIO, 255, 255, 255);

    // Initialize the RGB LED interface (RMT configuration)
    esp_err_t err = brainLED.init();
    if (err) {
        printf("LED couldn't be initialized.\n");
        printf("%s\n", esp_err_to_name(err));
        return;
    }

    // Turn on the LED with the initial color
    err = brainLED.turnOn();
    if (err) {
        printf("LED couldn't be turned on.\n");
        printf("%s\n", esp_err_to_name(err));
    }

    // Keep it on for 500ms
    vTaskDelay(pdMS_TO_TICKS(500));

    // Turn off
    brainLED.turnOff();
    vTaskDelay(pdMS_TO_TICKS(500));

    while (true) {
        brainLED.setColor(10, 0, 20);
        brainLED.pulse(100);
        vTaskDelay(pdMS_TO_TICKS(100));

        brainLED.setColor(0, 20, 10);
        brainLED.pulse(200);
        vTaskDelay(pdMS_TO_TICKS(200));

        brainLED.setColor(0, 50, 20);
        brainLED.pulse(300);
        vTaskDelay(pdMS_TO_TICKS(300));

        brainLED.setColor(10, 0, 10);
        brainLED.pulse(400);
        vTaskDelay(pdMS_TO_TICKS(400));

        brainLED.setColor(66, 33, 22);
        brainLED.pulse(500);
        vTaskDelay(pdMS_TO_TICKS(500));

        brainLED.setColor(10, 33, 77);
        brainLED.pulse(600);
        vTaskDelay(pdMS_TO_TICKS(600));

        brainLED.setColor(66, 33, 11);
        brainLED.pulse(700);
        vTaskDelay(pdMS_TO_TICKS(700));

        brainLED.setColor(0, 0, 33);
        brainLED.pulse(800);
        vTaskDelay(pdMS_TO_TICKS(800));

        brainLED.setColor(99, 66, 33);
        brainLED.pulse(900);
        vTaskDelay(pdMS_TO_TICKS(900));

        brainLED.setColor(5, 5, 5);
        brainLED.pulse(1000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
