/*******************************************************************************
 * @file debugger_example.cpp
 * @brief Contains the main file to show how to use the Debugger class.
 *
 * @version 0.2.2
 * @date 2024-08-29
 * @author emmanuel@sense-ai.co, Sense AI.
 *******************************************************************************
 *******************************************************************************/

#include "data_logging_sense.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

extern "C" void app_main() {
    Debugger debug(true);
    debug.println("Hola Mundo");
    debug.print("Presenting print method\n");
    uint8_t value0 = 0;
    debug.println(value0);
    uint16_t value1 = 1;
    debug.println(value1);
    uint32_t value2 = 2;
    debug.println(value2);
    uint64_t value3 = 3;
    debug.println(value3);
    int8_t value4 = 4;
    debug.println(value4);
    int16_t value5 = 5;
    debug.println(value5);
    int32_t value6 = 6;
    debug.println(value6);
    int64_t value7 = 7;
    debug.println(value7);
    char value8 = '8';
    debug.println(value8);

    debug.printf("formated %d, %u, %c\n", value4, value0, value8);

    uint32_t counter = 0;

    while (true) {
        debug.println(++counter);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
