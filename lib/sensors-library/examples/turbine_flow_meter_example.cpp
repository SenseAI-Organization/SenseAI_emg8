/*******************************************************************************
 * @file TurbineFlowMeter.hpp
 * @brief Contains the use example of TurbineFlowMeter class.
 *
 * Call updateFlow at least 1 time per second and read variables.
 *
 * @version v0.1.1
 * @date 2025-15-05
 * @author mauricio@sense-ai.co, Sense AI
 *******************************************************************************/

#include "turbine_flow_meter.hpp"

constexpr gpio_num_t kFlowSensorPin = GPIO_NUM_16;
const float kCalibrationFactor = 47.2;

TurbineFlowMeter flowSensor(kFlowSensorPin, kCalibrationFactor);

extern "C" void app_main() {
    if (flowSensor.init() != ESP_OK) {
        printf("TurbineFlowMeter Failed to initialize flow sensor.");
        return;
    }

    while (true) {
        /* Correct order of use:
         * 1. Run measure() at least 1 time/second in a loop.
         * 2. After 1 second measure() returns ESP_OK, means data is ready.
         * 3. Read values.
         */

        if (flowSensor.measure() == ESP_OK) {
            uint32_t currentTime = esp_timer_get_time() / 1000;
            float flowRate = flowSensor.getFlowRate();
            float totalLitres = flowSensor.getTotalLitres();
            uint16_t pulsesElapsed = flowSensor.getPulseCount();

            printf("Time: %lu ms", currentTime);
            printf("Raw Pulses: %u", pulsesElapsed);
            printf("Flow rate: %.2f L/min", flowRate);
            printf("Litres total: %.2f L", totalLitres);
        }

        /* note:  you are free to use measuse() without an if(), just make sure
         * to run this function preferably once/second. Also you can get any
         * value at any time, you might get a second old value or the actual.
         */

        vTaskDelay(pdMS_TO_TICKS(1000));  // can be any delay below 1 second
    }
}
