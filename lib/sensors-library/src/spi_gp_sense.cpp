/*******************************************************************************
 * @file spi_gp_sense.cpp
 * @brief Contains methods to handle SPI for general purpose for ESP32 SoCs as
 * part of the Sense Ecosystem.
 *
 * @version 0.1.0
 * @date 2025-04-22
 * @author Mateo R.B. (mateor@sense-ai.co)

 *******************************************************************************
 *******************************************************************************/
#include "spi_gp_sense.hpp"

// Base constructor
SPI::SPI(SpiMode mode, spi_host_device_t host, gpio_num_t mosiPin,
         gpio_num_t misoPin, gpio_num_t sclkPin)
    : mode_(mode),
      host_(host),
      mosiPin_(mosiPin),
      misoPin_(misoPin),
      sclkPin_(sclkPin) {};

SPI::~SPI() {
    deinit();
}

esp_err_t SPI::init(void) {
    esp_err_t err;
    switch (mode_) {
        case SpiMode::kMaster:
            err = initMaster();
            break;
        case SpiMode::kSlave:
            err = initSlave();
            break;
        default:
            err = ESP_ERR_INVALID_ARG;
            break;
    }

    return err;
}

esp_err_t SPI::deinit() {
    esp_err_t err = ESP_OK;
    switch (mode_) {
        case SpiMode::kSlave:
            // For slave mode, free slave interface first
            err = spi_slave_free(host_);
            if (err != ESP_OK) {
                return err;
            }

            break;
        case SpiMode::kMaster:
            // For master mode, devices should be removed by the user
            // before calling deinit, but we still free the bus
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    // Free the SPI bus (common for both master and slave)
    err = spi_bus_free(host_);

    return err;
}

spi_host_device_t SPI::getHost(void) {
    return host_;
}

esp_err_t SPI::addDevice(gpio_num_t csPin, int clkSpeedHz, ClockMode clkMode,
                         spi_device_handle_t *_deviceHandler) {
    spi_device_interface_config_t deviceConfig = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = static_cast<uint8_t>(clkMode),
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = clkSpeedHz,
        .input_delay_ns = 0,
        .spics_io_num = csPin,
        .flags = 0,
        .queue_size = 7,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    esp_err_t err = spi_bus_add_device(host_, &deviceConfig, _deviceHandler);
    if (err != ESP_OK) {
        spi_bus_remove_device(*_deviceHandler);
    }

    return err;
}

esp_err_t SPI::removeDevice(spi_device_handle_t *_deviceHandler) {
    esp_err_t err = spi_bus_remove_device(*_deviceHandler);
    return err;
}

esp_err_t SPI::write(spi_device_handle_t *deviceHandler, uint8_t *dataToWrite,
                     size_t len) {
    esp_err_t err = spi_device_acquire_bus(*deviceHandler, portMAX_DELAY);
    if (err) {
        return err;
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = len * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = dataToWrite,
        .rx_buffer = nullptr
    };

    err = spi_device_transmit(*deviceHandler, &transaction);
    spi_device_release_bus(*deviceHandler);

    return err;
}

esp_err_t SPI::read(spi_device_handle_t *deviceHandler, uint8_t *dataReceiver,
                    size_t len) {
    esp_err_t err = spi_device_acquire_bus(*deviceHandler, portMAX_DELAY);
    if (err) {
        return err;
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = len * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = dataReceiver
    };

    err = spi_device_transmit(*deviceHandler, &transaction);
    spi_device_release_bus(*deviceHandler);

    return err;
}

esp_err_t SPI::transfer(spi_device_handle_t *deviceHandler, uint8_t *tx_data,
                        uint8_t *rx_data, size_t len) {
    esp_err_t err = spi_device_acquire_bus(*deviceHandler, portMAX_DELAY);
    if (err) {
        return err;
    }

    spi_transaction_t transaction = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = len * 8,  // bits
        .rxlength = len * 8,
        .user = nullptr,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    err = spi_device_transmit(*deviceHandler, &transaction);
    spi_device_release_bus(*deviceHandler);

    return err;
}

esp_err_t SPI::initMaster(void) {
    spi_bus_config_t busConfig = {
        .mosi_io_num = mosiPin_,
        .miso_io_num = misoPin_,
        .sclk_io_num = sclkPin_,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = false,
        .max_transfer_sz = 4096,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0  // Initialize intr_flags
    };

    esp_err_t err = spi_bus_initialize(host_, &busConfig, SPI_DMA_CH_AUTO);

    return err;
}

esp_err_t SPI::initSlave(void) {
    spi_bus_config_t busConfig = {
        .mosi_io_num = mosiPin_,
        .miso_io_num = misoPin_,
        .sclk_io_num = sclkPin_,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = false,
        .max_transfer_sz = 4096,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0  // Initialize intr_flags
    };

    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = asSlaveCsPin_,
        .flags = 0,
        .queue_size = 7,
        .mode = static_cast<uint8_t>(asSlaveClkMode_),
        // These are callbacks used when the slave starts (setup_cb) and when a
        // transaction occurs (trans_cb)
        .post_setup_cb = nullptr,
        .post_trans_cb = nullptr};

    esp_err_t err =
        spi_slave_initialize(host_, &busConfig, &slvcfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        spi_bus_free(host_);
    }

    return err;
}