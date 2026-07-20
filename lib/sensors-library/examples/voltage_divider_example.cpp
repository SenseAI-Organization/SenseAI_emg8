/*******************************************************************************
 * @file voltage_divider_example.cpp
 * @brief File to test the VoltageDivider class and the Battery class.
 *
 * @version 0.3.0
 * @date 2024-10-18
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "voltage_divider_sense.hpp"

// Define the pin and resistor values for the voltage divider.
// This example simulates a sensor connected with a 1:1 divider.
#define SENSOR_ADC_PIN GPIO_NUM_8
#define SENSOR_R1_OHMS 10000
#define SENSOR_R2_OHMS 10000

static const char* TAG = "VOLTAGE_DIVIDER_EXAMPLE";

static void sensorTask(void* pvParameters) {
    // --- 1. Create a VoltageDivider instance ---
    VoltageDivider sensor(SENSOR_ADC_PIN, SENSOR_R1_OHMS, SENSOR_R2_OHMS);

    // --- 2. Configure ADC settings ---
    // Set attenuation to read the full range of the divider's output.
    sensor.setAttenuation(ADC_ATTEN_DB_12);

    // --- 3. Initialize the sensor ---
    esp_err_t err = sensor.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Voltage divider initialization failed with error: %d", err);
        vTaskDelete(NULL);  // Abort task
        return;
    }
    ESP_LOGI(TAG, "Sensor 0x%X initialized successfully.", sensor.getID());

    // --- 4. Check for calibration support ---
    if (sensor.isCalibrationEnabled()) {
        ESP_LOGI(TAG, "ADC calibration is available and enabled.");
    } else {
        ESP_LOGW(TAG,
                 "ADC calibration is not available. Voltage will be estimated from raw "
                 "values.");
    }

    // --- 5. Main measurement loop ---
    while (true) {
        // Use the `read...()` methods for a convenient one-shot measurement and return.
        // This internally calls measure() and then returns the requested value.
        uint16_t inputVoltage = sensor.readInputVoltage();

        // After a read, you can still get the other values from the same measurement.
        uint16_t outputVoltage = sensor.getOutputVoltage();
        int rawValue = sensor.getValue();
        int calibratedMv = sensor.getCalibratedMv();

        ESP_LOGI(TAG, "Input: %u mV | Output: %u mV | Raw ADC: %d | Calibrated: %d mV",
                 inputVoltage, outputVoltage, rawValue, calibratedMv);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main() {
    xTaskCreate(sensorTask, "SensorTask", 4096, nullptr, 5, nullptr);
}
