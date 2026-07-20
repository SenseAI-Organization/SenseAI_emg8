/*******************************************************************************
 * @file simple_led_example.cpp
 * @brief File to test the LED class.
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

extern "C" void app_main() {
    const gpio_num_t kExternalLedPin = GPIO_NUM_38;
    LED basicLed(kExternalLedPin);

    esp_err_t err = basicLed.init();
    if (err) {
        printf("LED couldn't be initialized.\n");
        printf("%s\n", esp_err_to_name(err));
        // while(1);
    }

    err = basicLed.turnOn();
    if (err) {
        printf("LED couldn't be turned on.\n");
        printf("%s\n", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    basicLed.turnOff();
    vTaskDelay(pdMS_TO_TICKS(500));

    while (true) {
        basicLed.pulse(100);
        vTaskDelay(pdMS_TO_TICKS(100));
        basicLed.pulse(200);
        vTaskDelay(pdMS_TO_TICKS(200));
        basicLed.pulse(300);
        vTaskDelay(pdMS_TO_TICKS(300));
        basicLed.pulse(400);
        vTaskDelay(pdMS_TO_TICKS(400));
        basicLed.pulse(500);
        vTaskDelay(pdMS_TO_TICKS(500));
        basicLed.pulse(600);
        vTaskDelay(pdMS_TO_TICKS(600));
        basicLed.pulse(700);
        vTaskDelay(pdMS_TO_TICKS(700));
        basicLed.pulse(800);
        vTaskDelay(pdMS_TO_TICKS(800));
        basicLed.pulse(900);
        vTaskDelay(pdMS_TO_TICKS(900));
        basicLed.pulse(1000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
