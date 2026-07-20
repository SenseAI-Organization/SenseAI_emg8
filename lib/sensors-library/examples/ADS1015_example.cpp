/*******************************************************************************
 * @file ADS1015_example.cpp
 * @brief Example Usage for the ADS1015 class reading two ADS1015 devices.
 *
 * This device is a four channel ADC with 12-bit resolution and a programmable
 * gain amplifier. The device can be configured to use one of four I2C addresses.
 * 
 * @version v0.0.1
 * @date 2025-01-16
 * @author daniel@sense-ai.co, Sense AI
 *******************************************************************************/

#include <mutex>
#include <cstring>
#include "esp_task_wdt.h"
#include "ADS1015.hpp"
#include "switch_sense.hpp"

constexpr gpio_num_t kSDA1 = GPIO_NUM_8;
constexpr gpio_num_t kSCL1 = GPIO_NUM_18;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);

constexpr gpio_num_t kSDA0 = GPIO_NUM_35;
constexpr gpio_num_t kSCL0 = GPIO_NUM_36;
I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 400000, false);

struct AdcTaskParams {
    const char* name;
    I2C* i2c;
    int startChannel;
};

struct sensorData {
    uint16_t channels[8];
};
sensorData sData;

uint8_t buffer[2];

extern "C" void app_main() {
    // Initialize I2C
    i2c0.init();
    i2c1.init();

    ADS1015 ads1(i2c0, ADS1015::ADS111X_Address::ADS111X_ADDR_GND);
    ADS1015 ads2(i2c1, ADS1015::ADS111X_Address::ADS111X_ADDR_GND);

    if (ads1.checkForDevice() != ESP_OK){
        printf("ADS1 not found\n");
    }
    if (ads2.checkForDevice() != ESP_OK){
        printf("ADS2 not found\n");
    }

    while (true) {
        for (int i = 0; i < 4; i++) {
            uint16_t value1 = ads1.readSingleEndedSigned(i);
            uint16_t value2 = ads2.readSingleEndedSigned(i);
            printf("%d %d ", value1, value2);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(1));  // Delay 1ms
    }
}