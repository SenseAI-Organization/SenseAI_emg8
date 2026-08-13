/*******************************************************************************
 * @file spi_gp_basic_example.cpp
 * @brief Contains the test example for spi-general purpose using espidf. The
 * example needs an ADXL345 to work.
 *
 * @version 0.1.0
 * @date 2025-04-22
 * @author mateor@sense-ai.co - Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "spi_gp_sense.hpp"

constexpr gpio_num_t kCsPin = GPIO_NUM_10;   // CS (White)
constexpr gpio_num_t kSclPin = GPIO_NUM_36;  // SCL (Blue)
constexpr gpio_num_t kSdoPin = GPIO_NUM_37;  // MOSI (Yellow)
constexpr gpio_num_t kSdaPin = GPIO_NUM_35;  // MISO (Green)

constexpr int clkSpeed = 1000000;  // 1MHz
SPI::ClockMode kClockMode = SPI::ClockMode::kMode3;

SPI::SpiMode kSpiGpMode = SPI::SpiMode::kMaster;
spi_host_device_t kSpiHost = SPI2_HOST;

SPI spiMaster(kSpiGpMode, kSpiHost, kSdoPin, kSdaPin, kSclPin);

extern "C" void app_main() {
    esp_err_t err = spiMaster.init();
    if (err) {
        printf("SPI error while init: %s\n", esp_err_to_name(err));
    }

    // After SPI initialization a device must be added in order to use it
    spi_device_handle_t
        accel345;  // Handle for the ADXL345 device
    err = spiMaster.addDevice(kCsPin, clkSpeed, kClockMode, &accel345);
    if (err) {
        printf("SPI error while attaching a device to the bus: %s\n",
               esp_err_to_name(err));
    }

    uint8_t txBuffer = 0x00;  // ADXL345 ID register

    while (1) {
        uint8_t rxBuffer = 0;  // To store the device ID

        err = spiMaster.transfer(&accel345, &txBuffer, &rxBuffer, 1);
        if (!err) {
            printf("Device ID: %2X\n", rxBuffer);
        } else {
            printf("SPI error during transaction: %s\n", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}