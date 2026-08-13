/*******************************************************************************
 * main.cpp
 * 
 * Main program for testing sensor libraries.
 * Sense-AI
********************************************************************************
*******************************************************************************/
#include <mutex>
#include <cstring>
#include "esp_task_wdt.h"
#include "ADS1015.hpp"
#include "ICM42605.hpp"

constexpr gpio_num_t mosiPin = GPIO_NUM_11;
constexpr gpio_num_t misoPin = GPIO_NUM_13;
constexpr gpio_num_t sclkPin = GPIO_NUM_12;
constexpr gpio_num_t csPin = GPIO_NUM_10;
SPI spi(SPI1_HOST, mosiPin, misoPin, sclkPin, csPin, 1000000);

uint8_t Ascale = ICM42605::RegistersBank2::AFS_2G,
        Gscale = ICM42605::RegistersBank2::GFS_250DPS,
        AODR = ICM42605::RegistersBank2::AODR_1000Hz, 
        GODR = ICM42605::RegistersBank2::GODR_1000Hz;

int16_t ICM42605Data[7];        // Stores the 16-bit signed sensor output

struct AdcTaskParams {
    const char* name;
    I2C* i2c;
    int startChannel;
};

struct sensorData {
    uint16_t channels[8];
};
sensorData sData;

struct imuData {
    uint16_t channels[5];
};
imuData mData;

uint16_t temperature;
uint8_t buffer[2];

extern "C" void app_main() {
    // Initialize SPI
    //spi.init();

    ICM42605 imu(spi, csPin);
    int8_t statusIMU = imu.begin(Ascale, Gscale, AODR, GODR);

    if (statusIMU < 0) {
        printf("IMU initialization unsuccessful\n");
        printf("Check IMU wiring or try cycling power\n");
        printf("Status: ");
        printf("%u\n", statusIMU);
      }

    while (true) {
        if (statusIMU > 0) {
            // Read sensor data
            imu.readSensor(ICM42605Data);

            // Print all measurements with a space between them
            for (int i = 0; i < 7; ++i) {
                printf("%d ", ICM42605Data[i]);
            }
            printf("\n");

            vTaskDelay(pdMS_TO_TICKS(1));  // Delay 100ms
        } else {
            printf("NO IMU DETECTED\n");
            vTaskDelay(pdMS_TO_TICKS(5000));  // Delay 1s
        }
    }
}