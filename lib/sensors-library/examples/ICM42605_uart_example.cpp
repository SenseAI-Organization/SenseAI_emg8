/*******************************************************************************
 * @file ICM42605_uart_example.cpp
 * @brief Read ICM42605 data and print through UART for debug.
 *
 * Toggle `kUseSpi` to select SPI or I2C transport.
 *******************************************************************************/

#include <cstdio>
#include <string>

#include "ICM42605.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_sense.hpp"

namespace {

constexpr bool kUseSpi = true;

// IMU SPI wiring on your board.
constexpr gpio_num_t kImuMosi = GPIO_NUM_35;
constexpr gpio_num_t kImuSck = GPIO_NUM_36;
constexpr gpio_num_t kImuMiso = GPIO_NUM_37;
constexpr gpio_num_t kImuCs = GPIO_NUM_38;
constexpr gpio_num_t kImuInt = GPIO_NUM_16;

// I2C mode on the same physical pins.
constexpr gpio_num_t kImuSda = kImuMosi;
constexpr gpio_num_t kImuScl = kImuSck;
constexpr gpio_num_t kImuAd0 = kImuMiso;

constexpr i2c_port_t kI2CPort = I2C_NUM_0;
constexpr uint32_t kI2CFrequencyHz = 400000;
constexpr bool kUseInternalPullups = true;
constexpr uint8_t kImuI2CAddress = 0x69;

constexpr spi_host_device_t kSPIHost = SPI3_HOST;
constexpr int kSPIFrequencyHz = 1000000;

constexpr int kUartBaudRate = 115200;
constexpr uint16_t kUartBufferSize = 1024;

constexpr TickType_t kStartupDelay = pdMS_TO_TICKS(50);
constexpr TickType_t kSampleDelay = pdMS_TO_TICKS(100);

void uartWriteLine(UART& uart, const std::string& line) {
    esp_err_t err = uart.write(line + "\n");
    if (err != ESP_OK) {
        printf("UART write failed: %s\n", esp_err_to_name(err));
    }
}

esp_err_t configureI2cPins() {
    esp_err_t err = gpio_reset_pin(kImuCs);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_direction(kImuCs, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_level(kImuCs, 1);  // Keep CS high so IMU stays in I2C mode.
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
    err = gpio_set_level(kImuAd0, kImuI2CAddress == 0x69 ? 1 : 0);
    if (err != ESP_OK) {
        return err;
    }

    return gpio_set_direction(kImuInt, GPIO_MODE_INPUT);
}

void streamSamples(UART& uart, ICM42605& imu) {
    while (true) {
        esp_err_t err = imu.measure();
        if (err != ESP_OK) {
            uartWriteLine(uart, std::string("ERR: measure() failed: ") + esp_err_to_name(err));
            vTaskDelay(kSampleDelay);
            continue;
        }

        char report[128] = {};
        err = imu.getReport(report);
        if (err != ESP_OK) {
            uartWriteLine(uart,
                          std::string("ERR: getReport() failed: ") + esp_err_to_name(err));
            vTaskDelay(kSampleDelay);
            continue;
        }

        uartWriteLine(uart, std::string("ICM42605,") + report);
        vTaskDelay(kSampleDelay);
    }
}

void runSpi(UART& uart) {
    uartWriteLine(uart, "ICM42605 UART debug example - SPI mode");
    uartWriteLine(uart, "Format: ICM42605,ax,ay,az,gx,gy,gz,temp");

    SPI spi(SPI::SpiMode::kMaster, kSPIHost, kImuMosi, kImuMiso, kImuSck);
    esp_err_t err = spi.init();
    if (err != ESP_OK) {
        uartWriteLine(uart, std::string("ERR: SPI init failed: ") + esp_err_to_name(err));
        return;
    }

    ICM42605 imu(spi, kImuCs, kSPIFrequencyHz);
    vTaskDelay(kStartupDelay);

    err = imu.init();
    if (err != ESP_OK) {
        uartWriteLine(uart, std::string("ERR: ICM42605 SPI init failed: ") +
                                esp_err_to_name(err));
        return;
    }

    uartWriteLine(uart, "OK: ICM42605 initialized on SPI");
    streamSamples(uart, imu);
}

void runI2c(UART& uart) {
    uartWriteLine(uart, "ICM42605 UART debug example - I2C mode");
    uartWriteLine(uart, "Format: ICM42605,ax,ay,az,gx,gy,gz,temp");

    esp_err_t err = configureI2cPins();
    if (err != ESP_OK) {
        uartWriteLine(uart,
                      std::string("ERR: I2C pin config failed: ") + esp_err_to_name(err));
        return;
    }

    I2C i2c(kI2CPort, kImuSda, kImuScl, kI2CFrequencyHz, kUseInternalPullups);
    vTaskDelay(kStartupDelay);

    err = i2c.init();
    if (err != ESP_OK) {
        uartWriteLine(uart, std::string("ERR: I2C init failed: ") + esp_err_to_name(err));
        return;
    }

    ICM42605 imu(i2c, kImuI2CAddress);
    err = imu.init();
    if (err != ESP_OK) {
        uartWriteLine(uart, std::string("ERR: ICM42605 I2C init failed: ") +
                                esp_err_to_name(err));
        return;
    }

    uartWriteLine(uart, "OK: ICM42605 initialized on I2C");
    streamSamples(uart, imu);
}

}  // namespace

extern "C" void app_main(void) {
    UART uart(kUartBaudRate, kUartBufferSize, false);
    esp_err_t err = uart.init();
    if (err != ESP_OK) {
        printf("UART init failed: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if constexpr (kUseSpi) {
        runSpi(uart);
    } else {
        runI2c(uart);
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
