/*******************************************************************************
 * @file timer_example.cpp
 * @brief An example code that shows how to use the methods related to the
 * timer class.
 *
 * @version v0.1.0
 * @date 2025-26-02
 * @author emmanuel@sense-ai.co, Sense-AI
 *******************************************************************************
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "smart_sensor_sense.hpp"

#define LED_PIN GPIO_NUM_38

volatile uint8_t ledState = 0;
volatile uint8_t intFlag = 0;
volatile uint8_t counterMs = 0;

/**
 * @brief Timer callback function.
 *
 * This callback toggles an LED every time the timer alarm fires.
 */
void IRAM_ATTR timerCallback(void* arg) {
    ledState ^= 1;
    gpio_set_level(LED_PIN, ledState);
    intFlag = 1;
    counterMs = counterMs + 1;
}

extern "C" void app_main(void) {
    // -----------------------------
    // Configure LED GPIO for testing.
    // -----------------------------
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // --------------------------------------------------------
    // Create a Timer instance with a 100 ms period, priority 1,
    // and auto-reload enabled. (Default unit is milliseconds.)
    // --------------------------------------------------------
    Timer timer(100, 1, true);

    // Initialize the timer hardware.
    if (timer.init() != ESP_OK) {
        printf("Failed to initialize timer.\n");
        return;
    }

    // Configure the timer alarm with the current settings.
    if (timer.configure() != ESP_OK) {
        printf("Failed to configure timer.\n");
        return;
    }

    // Set the callback function to be executed on each alarm.
    timer.setCallback(timerCallback);

    // Start the timer counter.
    if (timer.start() != ESP_OK) {
        printf("Failed to start timer.\n");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t counter = 0;

    // Reset the counter to 0.
    if (timer.reset() == ESP_OK) {
        printf("Counter reset to 0.\n");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (timer.getCounterMs(&counter) == ESP_OK) {
        printf("Counter after reset and 50ms: %lu\n", counter);
    }

    // ----------------------------------------------------------
    // Test reconfiguration using frequency and period methods.
    // Set frequency to 200 Hz (which internally recalculates the alarm count).
    // ----------------------------------------------------------
    if (timer.setFrequency(200) == ESP_OK) {
        printf("Timer reconfigured to 200 Hz.\n");
    }

    printf("Current frequency: %lu Hz\n", timer.getFrequency());

    // Now set the period to 500 ms.
    if (timer.setPeriodMs(500) == ESP_OK) {
        printf("Timer reconfigured to 500 ms period.\n");
    }

    printf("Current period: %lu ms\n", timer.getPeriodMs());

    vTaskDelay(pdMS_TO_TICKS(10000));

    // -----------------------------------------------------------
    // Test enabling/disabling interrupt callback execution.
    // Disable interrupts for 500 ms (the LED should not toggle during this time).
    // -----------------------------------------------------------
    timer.disableInterrupts();
    printf("Interrupts disabled.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Re-enable interrupts; the LED should start toggling again.
    timer.enableInterrupts();
    printf("Interrupts enabled.\n");

    // -----------------------------------------------------------
    // Main loop: periodically print the current counter value.
    // -----------------------------------------------------------
    int64_t time_since_wakeup = esp_timer_get_time(); // microseconds
    while (true) {
        if (intFlag) {
            int64_t currentTime = esp_timer_get_time() - time_since_wakeup;
            printf("%lld us - Count: %u\n", currentTime, counterMs);
            intFlag = false;
        }

        if (counterMs >= 50) {
            printf("Timer has been stoped!\n");
            counterMs = 0;
            timer.stop();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
