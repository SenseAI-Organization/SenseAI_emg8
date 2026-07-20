/*******************************************************************************
 * @file main.cpp
 * @brief BatteryManager example demonstrating comprehensive battery monitoring
 *
 * This example shows:
 * - Battery voltage and percentage reading with stabilization
 * - Charging state detection
 * - Power connection (Pgood) detection
 * - State-aware reporting
 *
 * @version 1.0.0
 * @date 2026-03-18
 * @author <isa@senseai.co>
 ******************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "BatteryManager.hpp"

/* Pin definitions - adjust these based on your hardware */
constexpr gpio_num_t batteryPin = GPIO_NUM_8;   // ADC pin for battery voltage divider
constexpr gpio_num_t chargingPin = GPIO_NUM_15;  // Digital pin: HIGH when charging
constexpr gpio_num_t pGoodPin = GPIO_NUM_16;  // Digital pin: HIGH when connected to power

/* Measurement interval in milliseconds */
constexpr uint16_t kMeasurementInterval = 2000;

/* Critical battery threshold (mV) */
constexpr uint16_t kBatteryThreshold = 2800;

/**
 * @brief Battery monitoring task
 */
static void batteryMonitorTask(void* pvParameters) {
    // Method 1: Use defaults with custom pins (simplest)
    BatteryManager batteryMgr(batteryPin, chargingPin, pGoodPin);
    
    // Initialize the battery manager
    esp_err_t err = batteryMgr.init();
    if (err != ESP_OK) {
        printf("ERROR: BatteryManager initialization failed (error: %d)\n", err);
        vTaskDelete(NULL);
        return;
    }
    
    printf("====================================\n");
    printf("BatteryManager initialized successfully\n");
    printf("====================================\n\n");
    
    // Main monitoring loop
    uint32_t iteration = 0;
    while (true) {
        // Perform measurement (reads voltage, charging state, power state)
        err = batteryMgr.measure();
        if (err != ESP_OK) {
            printf("ERROR: Measurement failed (error: %d)\n", err);
            vTaskDelay(pdMS_TO_TICKS(kMeasurementInterval));
            continue;
        }
        
        // Get measurement data
        uint16_t voltage = batteryMgr.getVoltage();
        uint8_t percentage = batteryMgr.getPercentage();
        bool charging = batteryMgr.isCharging();
        bool connected = batteryMgr.isConnected();
        
        // Display header every 10 iterations for readability
        if (iteration % 10 == 0) {
            printf("\n");
            printf("====================================\n");
            printf("  Battery Monitoring System\n");
            printf("====================================\n");
        }
        
        // Display current readings
        printf("Iteration %lu | State: %s\n", iteration, batteryMgr.getStateString());
        printf("  Voltage:    %4u mV (raw: %4u mV)\n", 
               voltage, batteryMgr.getRawVoltage());
        printf("  Charge:     %3u%% (raw: %3u%%)\n", 
               percentage, batteryMgr.getRawPercentage());
        printf("  Charging:   %s\n", charging ? "YES" : "NO");
        printf("  Connected:  %s\n", connected ? "YES" : "NO");
        
        // Check for critical conditions
        if (batteryMgr.isCriticallyLow(kBatteryThreshold)) {
            printf("    WARNING: Battery critically low!\n");
        }
        
        if (batteryMgr.isFullyCharged(95)) {
            printf("   Battery fully charged\n");
        }
        
        printf("------------------------------------\n");
        
        // Optional: Get full report every 20 iterations
        if (iteration % 20 == 0) {
            char report[256];
            batteryMgr.getReport(report);
            printf("\n%s\n\n", report);
        }
        
        iteration++;
        vTaskDelay(pdMS_TO_TICKS(kMeasurementInterval));
    }
}

/**
 * @brief Application entry point
 */
extern "C" void app_main(void) {
    printf("\n\n");
    printf("====================================\n");
    printf("  BatteryManager Example\n");
    printf("  Version 1.0.0\n");
    printf("====================================\n\n");
    
    // Create battery monitoring task
    xTaskCreate(batteryMonitorTask, 
                "BatteryMonitor", 
                4096,           // Stack size 
                NULL,           // Parameters
                5,              // Priority
                NULL);          // Task handle
}