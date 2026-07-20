/*******************************************************************************
 * @file DYP_A22YYMW_example.cpp
 * @brief File to test the DYP_A22YYMW class.
 *
 * @version v0.1.1
 * @date 2025-02-10
 * @author Isabella Garcia isa@sense-ai.co
 *******************************************************************************/

#include "DYP_A22YYMW.hpp"

constexpr gpio_num_t kTrigPin = GPIO_NUM_17;
constexpr gpio_num_t kEchoPin = GPIO_NUM_18;

DYP_A22YYMW sensor(kTrigPin, kEchoPin);

extern "C" void app_main() {
    if (sensor.init() != ESP_OK) {
        printf("DYP_A22YYMW Failed to initialize distance sensor.");
        return;
    }

    int64_t startTime = esp_timer_get_time();

    while (true) {
        float distance = 0;
        esp_err_t err = sensor.readDistance(&distance);

        uint32_t currentTime = (esp_timer_get_time() - startTime) / 1000;

        if (err == ESP_OK) {
            printf("time: %lu ms, distance: %.2f cm\n", currentTime, distance);
        } else if (err == ESP_ERR_TIMEOUT) {
            printf("Timeout error\n");
        } else {
            printf("Error reading sensor: %d\n", err);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Wait at least 10 ms
    }
}
