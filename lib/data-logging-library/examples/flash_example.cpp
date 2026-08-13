/*******************************************************************************
 * @file flash_example.cpp
 * @brief Contains the main file to show how to use the FlashStorage class.
 *
 * @version 0.2.2
 * @date 2024-08-29
 * @author emmanuel@sense-ai.co, Sense AI.
 *******************************************************************************
 *******************************************************************************/

#include "flash_sense.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

extern "C" void app_main() {
    FlashStorage flash("Storage");

    esp_err_t err = flash.init();
    if (err) {
        printf(">> Flash storage couldn't be initialized! (%i)\n", err);
    }

    uint16_t valueToSave = 0xACDC;
    err = flash.put("uint16", valueToSave);
    if (err) {
        printf(">> Value couldn't be saved on flash! (%i)\n", err);
    } else {
        printf("Value saved on flash: %X\n", valueToSave);
    }

    vTaskDelay(pdMS_TO_TICKS(3333));

    uint16_t loadedValue = 0;
    err = flash.get("uint16", loadedValue);
    if (err) {
        printf(">> Value couldn't be saved on flash! (%i)\n", err);
    } else {
        printf("Value loaded from flash: %X\n", loadedValue);
    }

    /**
     * This should be used after all the writing and readings operations
     */
    err = flash.commit();
    if (err) {
        printf(">> Couldn't commit changes on flash! (%i)\n", err);
    }

    printf("Changes saved succesfully\n");

    /**
     * This should be used after the commit to secure the changes
     */
    flash.end();

    printf("Flash program ended");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
