/*******************************************************************************
 * @file battery_example.cpp
 * @brief Example demonstrating the Battery class usage for voltage monitoring.
 *
 * This example shows how to initialize and read battery voltage and charge
 * percentage using the Battery class with a voltage divider circuit.
 *
 * @version 0.3.0
 * @date 2025-08-01
 * @author emmanuel@sense-ai.co, Sense AI
 ******************************************************************************/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "voltage_divider_sense.hpp"

/* ADC pin connected to the battery voltage divider */
#define BATTERY_ADC_PIN GPIO_NUM_8

/* Measurement interval in milliseconds */
#define MEASUREMENT_INTERVAL_MS 2000

static void batteryTask(void* pvParameters) {
    /* Create Battery instance with default 1:1 voltage divider and LiPo range (3.2V-4.2V)
     * For custom voltage dividers: Battery battery(pin, R1, R2, minVoltage, maxVoltage)
     */
    Battery battery(BATTERY_ADC_PIN);

    /* Configure ADC: 12dB attenuation for 0-3.3V range, 12-bit resolution */
    battery.setAttenuation(ADC_ATTEN_DB_12);
    battery.setBitResolution(ADC_BITWIDTH_DEFAULT);

    /* Initialize the ADC and calibration */
    esp_err_t err = battery.init();
    if (err != ESP_OK) {
        printf("ERROR: Battery sensor initialization failed (error: %d)\n", err);
        vTaskDelete(NULL);
        return;
    }

    printf("Battery sensor initialized successfully (ID: 0x%X)\n", battery.getID());

    if (battery.isCalibrationEnabled()) {
        printf("ADC calibration enabled\n");
    } else {
        printf("WARNING: ADC calibration unavailable, using raw values\n");
    }

    /* Main measurement loop */
    while (true) {
        battery.measure();

        printf("-------------------------------\n");
        printf("Charge: %3u%% | Voltage: %4u mV\n", battery.getChargePercentage(),
               battery.getInputVoltage());
        printf("  Raw ADC:     %d\n", battery.getValue());
        printf("  Calibrated:  %d mV\n", battery.getCalibratedMv());

        char report[128];
        battery.getReport(report);
        printf("  Report:      %s\n", report);

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}

extern "C" void app_main(void) {
    xTaskCreate(batteryTask, "BatteryTask", 4096, NULL, 5, NULL);
}
