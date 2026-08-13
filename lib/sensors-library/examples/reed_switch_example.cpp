/*******************************************************************************
 * @file reed_switch_example.cpp
 * @brief Example of using the Switch class with a Reed Switch in polling mode.
 *
 * This example demonstrates:
 * - Configuring a Switch as Normally Closed.
 * - Polling the switch state every 20ms.
 * - Printing the state to the console.
 *
 * @version 0.1.0
 * @date 2025-11-27
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "switch_sense.hpp"

// Configuration constants
constexpr gpio_num_t kReedSwitchPin = GPIO_NUM_17;  // Adjust pin as needed
constexpr bool kReedExternalResistor = false;  // Set to true if using external resistor

extern "C" void app_main() {
    printf("\n\n<< Reed Switch Polling Example >>\n\n");

    // Instantiate Switch with requested configuration
    // Mode: Normally Closed
    // Resistor: Defined by kReedExternalResistor
    Switch reedSwitch(kReedSwitchPin, Switch::SwitchMode::kNormallyClosed,
                      kReedExternalResistor);

    // Initialize the switch
    esp_err_t err = reedSwitch.init();
    if (err != ESP_OK) {
        printf("Error initializing Reed Switch: %d\n", err);
        return;
    }

    printf("Reed Switch Initialized on GPIO %d\n", kReedSwitchPin);
    printf("Sampling at 20ms...\n");

    while (true) {
        // Sample the switch state
        // readState() returns the logical state based on the configuration
        bool state = reedSwitch.readState();

        // Print the reading
        printf("Level: %d\n", state);

        // Delay for 20ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
