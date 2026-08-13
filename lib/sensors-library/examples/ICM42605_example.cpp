/*******************************************************************************
 * @file ICM42605_example.cpp
 * @brief Minimal example for the ICM42605 using either SPI or I2C.
 *
 * Toggle `kUseSpi` to switch transports.
 *
 * For this board, the ICM-42605 pins are shared between SPI and I2C:
 * - SDA uses the same line as SDIO/MOSI
 * - SCL uses the same line as SCLK
 * - AD0 uses the same line as SDO/MISO
 * - CS must be held high in I2C mode
 *******************************************************************************/

#include <cstdio>

#include "ICM42605.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr bool kUseSpi = false;

// IMU SPI wiring on the current board.
constexpr gpio_num_t kImuMosi = GPIO_NUM_35;
constexpr gpio_num_t kImuSck = GPIO_NUM_36;
constexpr gpio_num_t kImuMiso = GPIO_NUM_37;
constexpr gpio_num_t kImuCs = GPIO_NUM_38;
constexpr gpio_num_t kImuInt = GPIO_NUM_16;

// In I2C mode the same physical IMU pins are reused.
constexpr gpio_num_t kImuSda = kImuMosi;
constexpr gpio_num_t kImuScl = kImuSck;
constexpr gpio_num_t kImuAd0 = kImuMiso;

constexpr i2c_port_t kI2CPort = I2C_NUM_0;
constexpr uint32_t kI2CFrequencyHz = 400000;
constexpr bool kUseInternalPullups = true;
constexpr uint8_t kIMUI2CAddress = 0x69;

constexpr spi_host_device_t kSPIHost = SPI3_HOST;
constexpr int kSPIFrequencyHz = 1000000;

constexpr TickType_t kStartupDelay = pdMS_TO_TICKS(50);
constexpr TickType_t kSampleDelay = pdMS_TO_TICKS(100);

esp_err_t configureI2cPins() {
    esp_err_t err = gpio_reset_pin(kImuCs);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_direction(kImuCs, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_level(kImuCs, 1);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_reset_pin(kImuAd0);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_direction(kImuAd0, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_level(kImuAd0, kIMUI2CAddress == 0x69 ? 1 : 0);
    if (err != ESP_OK) {
        return err;
    }

    return gpio_set_direction(kImuInt, GPIO_MODE_INPUT);
}

void printSample(ICM42605& imu) {
    char report[128] = {};
    esp_err_t err = imu.measure();
    if (err != ESP_OK) {
        printf("measure() failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = imu.getReport(report);
    if (err != ESP_OK) {
        printf("getReport() failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("ax,ay,az,gx,gy,gz,temp = %s", report);
}

void runSpiExample() {
    printf("\nICM42605 example starting in SPI mode\n");
    printf("MOSI=%d SCK=%d MISO=%d CS=%d INT=%d\n", kImuMosi, kImuSck, kImuMiso, kImuCs,
           kImuInt);

    SPI spi(SPI::SpiMode::kMaster, kSPIHost, kImuMosi, kImuMiso, kImuSck);
    esp_err_t err = spi.init();
    if (err != ESP_OK) {
        printf("SPI init failed: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ICM42605 imu(spi, kImuCs, kSPIFrequencyHz);
    vTaskDelay(kStartupDelay);

    err = imu.init();
    if (err != ESP_OK) {
        printf("ICM42605 SPI init failed: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    printf("ICM42605 ready over SPI\n");
    while (true) {
        printSample(imu);
        vTaskDelay(kSampleDelay);
    }
}

void runI2cExample() {
    printf("\nICM42605 example starting in I2C mode\n");
    printf("SDA=%d SCL=%d AD0=%d CS=%d INT=%d ADDR=0x%02X\n", kImuSda, kImuScl, kImuAd0,
           kImuCs, kImuInt, kIMUI2CAddress);

    esp_err_t err = configureI2cPins();
    if (err != ESP_OK) {
        printf("I2C mode pin setup failed: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    I2C i2c(kI2CPort, kImuSda, kImuScl, kI2CFrequencyHz, kUseInternalPullups);
    vTaskDelay(kStartupDelay);

    err = i2c.init();
    if (err != ESP_OK) {
        printf("I2C init failed: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ICM42605 imu(i2c, kIMUI2CAddress);
    err = imu.init();
    if (err != ESP_OK) {
        printf("ICM42605 I2C init failed: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    printf("ICM42605 ready over I2C\n");
    while (true) {
        printSample(imu);
        vTaskDelay(kSampleDelay);
    }
}

}  // namespace

extern "C" void app_main(void) {
    if constexpr (kUseSpi) {
        runSpiExample();
    } else {
        runI2cExample();
    }
}
