/*******************************************************************************
 * @file switch_example.cpp
 * @brief File to test the Switch class.
 * 
 * @version 0.2.1
 * @date 2024-06-18
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "switch_sense.hpp"

volatile uint8_t buttonFlag = 0;

void userButtonCallback(void* arg);

static void userButtonTask(void* pvParameters) {
    const bool kExternalPullUp = true;
    Switch userButton(
        GPIO_NUM_9,
        Switch::SwitchMode::kNormallyOpen,
        kExternalPullUp
    );

    esp_err_t err = userButton.configureInterrupt(GPIO_INTR_POSEDGE, 
                                                  userButtonCallback, nullptr);
    if (err) {
        printf("Interrupt configuration error %d", err);
    }

    // Verify task size in case of error while running the program.
    userButton.startHandlerTask("ButtonHandlerTask", 5, 2048);

    err = userButton.init();
    if (err) {
        printf("User button init error %d\n", err);
    }

    /* On this main loop there are some examples on how the button can be used
     */
    while (true) {
        bool buttonState = userButton.readState();
        printf("User button state: %d\n", buttonState);

        if (userButton.isPressed()) {
            printf("User button Being pressed!\n");
        }

        if (buttonFlag) {
            printf("User button ISR called\n");
            buttonFlag = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main() {
    xTaskCreate(userButtonTask, "UserButtonTask", 4096, NULL, 5, NULL);
}

/** 
 * The code written here is handled in the specified task.
 * It can be as complex as the task size allows it.
 */
void userButtonCallback(void* arg) {

    printf("Interrupt!\n");
    buttonFlag = 1;
}