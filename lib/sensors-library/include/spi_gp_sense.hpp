/*******************************************************************************
 * @file spi_gp_sense.hpp
 * @brief Contains methods to handle SPI for general purpose for ESP32 SoCs as
 * part of the Sense Ecosystem.
 *
 * @version 0.1.0
 * @date 2025-04-22
 * @author Mateo R.B. (mateor@sense-ai.co)

 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"

/******************************************************************************/
/*                                  SPI                                       */
/******************************************************************************/

/**
 * @class SPI
 *
 * @brief A class to manage the SPI peripheral for general purpose communication
 *
 * This class provides a comprehensive interface for SPI (Serial Peripheral
 * Interface) communication on ESP32 SoCs. It supports both master and slave(not
 * tested) modes with configurable clock modes and device management.
 */
class SPI {
public:
    /**
     * @enum SpiMode
     * @brief Enumeration for SPI operation mode
     */
    enum class SpiMode : uint8_t {
        kMaster = 0x00,  ///< SPI Master mode - controls the clock signal
        kSlave = 0x01,   ///< SPI Slave mode - receives clock from master
    };

    /**
     * @enum ClockMode
     * @brief Enumeration for SPI clock polarity and phase configuration
     * @note Clock modes define the relationship between clock polarity (CPOL)
     * and clock phase (CPHA):
     *       - Mode 0: CPOL=0, CPHA=0 - Clock idle low, data sampled on rising
     * edge
     *       - Mode 1: CPOL=0, CPHA=1 - Clock idle low, data sampled on falling
     * edge
     *       - Mode 2: CPOL=1, CPHA=0 - Clock idle high, data sampled on falling
     * edge
     *       - Mode 3: CPOL=1, CPHA=1 - Clock idle high, data sampled on rising
     * edge
     */
    enum class ClockMode : uint8_t {
        kMode0 = 0b00,  ///< Mode 0: CPOL=0, CPHA=0
        kMode1 = 0b01,  ///< Mode 1: CPOL=0, CPHA=1
        kMode2 = 0b10,  ///< Mode 2: CPOL=1, CPHA=0
        kMode3 = 0b11,  ///< Mode 3: CPOL=1, CPHA=1
    };
    
    /**
     * @brief Construct a new SPI object for general purpose communication
     *
     * @param mode SPI operation mode (Master or Slave)
     * @param host SPI host peripheral to use (SPI2_HOST or SPI3_HOST)
     * @param mosiPin GPIO pin number for Master Out Slave In (MOSI)
     * @param misoPin GPIO pin number for Master In Slave Out (MISO)
     * @param sclkPin GPIO pin number for Serial Clock (SCLK)
     *
     * @note CS (Chip Select) pins are configured per device when adding devices
     * in master mode
     */
    SPI(SpiMode mode, spi_host_device_t host, gpio_num_t mosiPin,
        gpio_num_t misoPin, gpio_num_t sclkPin);

    /**
     * @brief Destructor for the SPI object
     *
     * Automatically deinitializes the SPI peripheral and frees allocated
     * resources
     */
    ~SPI();

    /**
     * @brief Initialize the SPI peripheral with the parameters given in the
     * constructor
     *
     * @return ESP_OK on success, ESP_FAIL on parameter error or initialization
     * failure
     *
     * @note This function must be called before using any other SPI operations
     */
    esp_err_t init(void);

    /**
     * @brief Add a new device to the SPI master bus
     *
     * @param csPin GPIO pin number for Chip Select (CS) signal
     * @param clkSpeedHz Clock frequency in Hz (typically 1MHz to 80MHz)
     * @param clkMode Clock mode configuration (see ClockMode enum)
     * @param _deviceHandler Pointer to store the device handle for future
     * operations
     *
     * @return ESP_OK on success, ESP_FAIL on parameter error or device addition
     * failure
     *
     * @note Only available in master mode. Each device can have different clock
     * settings
     */
    esp_err_t addDevice(gpio_num_t csPin, int clkSpeedHz, ClockMode clkMode,
                        spi_device_handle_t *_deviceHandler);

    /**
     * @brief Remove a device from the SPI master bus
     *
     * @param _deviceHandler Pointer to the device handle to remove
     *
     * @return ESP_OK on success, ESP_FAIL on parameter error or device removal
     * failure
     *
     * @note Only available in master mode. Frees the device handle and
     * associated resources
     */
    esp_err_t removeDevice(spi_device_handle_t *_deviceHandler);

    /**
     * @brief Get the SPI host peripheral being used
     *
     * @return spi_host_device_t The SPI host (SPI2_HOST or SPI3_HOST)
     */
    spi_host_device_t getHost(void);

    /**
     * @brief Write data to a specific SPI device
     *
     * @param deviceHandler Pointer to the device handle
     * @param dataToWrite Pointer to the data buffer to transmit
     * @param len Number of bytes to write
     *
     * @return ESP_OK on success, ESP_FAIL on parameter error or transmission
     * failure
     *
     * @note Only available in master mode. CS signal is automatically managed
     */
    esp_err_t write(spi_device_handle_t *deviceHandler, uint8_t *dataToWrite,
                    size_t len);

    /**
     * @brief Read data from a specific SPI device
     *
     * @param deviceHandler Pointer to the device handle
     * @param dataReceiver Pointer to the buffer to store received data
     * @param len Number of bytes to read
     *
     * @return ESP_OK on success, ESP_FAIL on parameter error or reception
     * failure
     *
     * @note Only available in master mode. CS signal is automatically managed
     */
    esp_err_t read(spi_device_handle_t *deviceHandler, uint8_t *dataReceiver,
                   size_t len);

    /**
     * @brief Perform a full-duplex SPI transfer (simultaneous read and write)
     *
     * @param deviceHandler Pointer to the device handle
     * @param tx_data Pointer to the data buffer to transmit
     * @param rx_data Pointer to the buffer to store received data
     * @param len Number of bytes to transfer in both directions
     *
     * @return ESP_OK on success, ESP_FAIL on parameter error or transfer
     * failure
     *
     * @note Only available in master mode. CS signal is automatically managed.
     *       Data is transmitted and received simultaneously.
     */
    esp_err_t transfer(spi_device_handle_t *deviceHandler, uint8_t *tx_data,
                       uint8_t *rx_data, size_t len);

private:
    SpiMode mode_;            ///< SPI operation mode (Master/Slave)
    spi_host_device_t host_;  ///< SPI host peripheral (SPI2_HOST/SPI3_HOST)
    gpio_num_t mosiPin_;      ///< Master Out Slave In GPIO pin
    gpio_num_t misoPin_;      ///< Master In Slave Out GPIO pin
    gpio_num_t sclkPin_;      ///< Serial Clock GPIO pin

    // For slave mode configuration
    gpio_num_t asSlaveCsPin_;   ///< Chip Select pin when operating as slave
    ClockMode asSlaveClkMode_;  ///< Clock mode when operating as slave

    /**
     * @brief Initialize SPI in master mode
     *
     * @return ESP_OK on success, ESP_FAIL on initialization failure
     */
    esp_err_t initMaster(void);

    /**
     * @brief Initialize SPI in slave mode
     *
     * @return ESP_OK on success, ESP_FAIL on initialization failure
     */
    esp_err_t initSlave(void);

    /**
     * @brief Deinitialize the SPI peripheral and free resources
     *
     * @return ESP_OK on success, ESP_FAIL on deinitialization failure
     *
     * @note For slave mode, this will free both the slave interface and the
     * bus. For master mode, ensure all devices are removed before calling this
     * function.
     */
    esp_err_t deinit();
};
