/*******************************************************************************
 * main.cpp
 *
 * Main program for testing sensor libraries.
 * Sense-AI
 ********************************************************************************
 *******************************************************************************/
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "limit_switch_sense.hpp"

volatile uint8_t buttonFlag = 0;

void doorCallback(void* arg);

static void doorSwitchTask(void* pvParameters) {
    LimitSwitch doorSwitch(GPIO_NUM_5, LimitSwitch::SwitchMode::kNormallyClosed);

    doorSwitch.configure();

    esp_err_t err =
        doorSwitch.configureInterrupt(GPIO_INTR_POSEDGE, doorCallback, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE("DOOR_SWITCH", "Interrupt configuration error %d", err);
    }

    doorSwitch.startHandlerTask("ButtonHandlerTask", 5);

    err = doorSwitch.init();
    if (err != ESP_OK) {
        ESP_LOGE("DOOR_SWITCH", "Init error %d", err);
    }

    while (true) {
        int8_t doorState = doorSwitch.readState();
        ESP_LOGI("DOOR_SWITCH", "State: %d", doorState);
        if (doorSwitch.isPressed()) {
            ESP_LOGI("DOOR_SWITCH", "Being pressed!");
        }
        if (buttonFlag) {
            ESP_LOGI("DOOR_SWITCH", "ISR called");
            buttonFlag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

extern "C" void app_main() {
    xTaskCreate(doorSwitchTask, "DoorSwitchTask", 4096, NULL, 5, NULL);
}

void doorCallback(void* arg) {
    printf("Interrupt!\n");
}
