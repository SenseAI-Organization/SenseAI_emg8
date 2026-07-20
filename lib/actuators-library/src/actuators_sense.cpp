/*******************************************************************************
 * @file actuators_sense.cpp
 * @brief Contains the implementations to work with Actuator Interface and some
 * generic abstract actuators for ESP32 SoCs as part of the Sense Ecosystem.
 *
 * @version 0.1.2
 * @date 2025-10-1
 * @author emmanuel@sense-ai.co, mateor@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#include "actuators_sense.hpp"

Actuator::~Actuator() {
}

LED::LED(gpio_num_t pin) : pin_(pin), state_(LedState::kOff) {
}

LED::~LED() {
}

esp_err_t LED::init(void) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin_)) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin_),         // GPIO pin bit mask
        .mode = GPIO_MODE_OUTPUT,               // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,      // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE          // No interrupt
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t LED::pulse(uint32_t millisOn) {
    esp_err_t err = turnOn();
    if (err) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(millisOn));

    err = turnOff();
    if (err) {
        return err;
    }

    return ESP_OK;
}

esp_err_t LED::cycle(uint32_t millisOn) {
    return ESP_OK;
}

esp_err_t LED::turnOn(void) {
    return gpio_set_level(pin_, static_cast<uint32_t>(LedState::kOn));
}

esp_err_t LED::turnOff(void) {
    return gpio_set_level(pin_, static_cast<uint32_t>(LedState::kOff));
}

/******************************************************************************/
/*                                RGB                                         */
/******************************************************************************/

RGB::RGB(gpio_num_t gpioNum)
    : gpioNum_(gpioNum),
      redIntensity_(0),
      greenIntensity_(0),
      blueIntensity_(0),
      isOn_(false) {
}

RGB::RGB(gpio_num_t gpioNum, uint8_t red, uint8_t green, uint8_t blue)
    : gpioNum_(gpioNum),
      redIntensity_(red),
      greenIntensity_(green),
      blueIntensity_(blue),
      isOn_(false) {
}

RGB::~RGB() {
    turnOff();

    // Clean up RMT resources
    if (encoderHandle_) {
        rmt_del_encoder(encoderHandle_);
        encoderHandle_ = nullptr;
    }

    if (channelHandle_) {
        rmt_disable(channelHandle_);
        rmt_del_channel(channelHandle_);
        channelHandle_ = nullptr;
    }
}

esp_err_t RGB::init(void) {
    // Validate GPIO pin
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpioNum_)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize RMT channel
    esp_err_t err = initRmtChannel(&channelHandle_);
    if (err != ESP_OK) {
        return err;
    }

    // Enable RMT channel
    err = rmt_enable(channelHandle_);
    if (err != ESP_OK) {
        rmt_del_channel(channelHandle_);
        channelHandle_ = nullptr;
        return err;
    }

    // Create byte encoder for WS2812 protocol
    err = createCustomEncoder(&encoderHandle_);
    if (err != ESP_OK) {
        rmt_disable(channelHandle_);
        rmt_del_channel(channelHandle_);
        channelHandle_ = nullptr;
        return err;
    }

    // Initialize with LED off
    isOn_ = false;
    return updateHardware(0, 0, 0);
}

esp_err_t RGB::turnOn(void) {
    isOn_ = true;
    return updateHardware();
}

esp_err_t RGB::turnOff(void) {
    isOn_ = false;
    return updateHardware(0, 0, 0);
}

esp_err_t RGB::pulse(uint32_t millisOn) {
    esp_err_t err = turnOn();
    if (err) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(millisOn));

    return turnOff();
}

esp_err_t RGB::cycle(uint32_t millisOn) {
    turnOn();
    vTaskDelay(pdMS_TO_TICKS(millisOn));
    turnOff();
    vTaskDelay(pdMS_TO_TICKS(millisOn));
    return ESP_OK;
}

esp_err_t RGB::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    redIntensity_ = red;
    greenIntensity_ = green;
    blueIntensity_ = blue;

    return updateHardware();
}

bool RGB::isOn(void) const {
    return isOn_;
}

void RGB::getColor(uint8_t& red, uint8_t& green, uint8_t& blue) const {
    red = redIntensity_;
    green = greenIntensity_;
    blue = blueIntensity_;
}

esp_err_t RGB::updateHardware(void) {
    if (isOn_) {
        return updateHardware(redIntensity_, greenIntensity_, blueIntensity_);
    } else {
        return updateHardware(0, 0, 0);
    }
}

esp_err_t RGB::updateHardware(uint8_t red, uint8_t green, uint8_t blue) {
    if (!channelHandle_ || !encoderHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    // WS2812 expects GRB order
    uint8_t colorData[3] = {green, red, blue};

    rmt_transmit_config_t txConfig = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,         // End-of-transmission level: low
            .queue_nonblocking = 0  // Blocking transmission
        }};

    // Transmit color data
    esp_err_t err = rmt_transmit(channelHandle_, encoderHandle_, colorData,
                                 sizeof(colorData), &txConfig);
    if (err != ESP_OK) {
        return err;
    }

    // Wait for transmission to complete
    err = rmt_tx_wait_all_done(channelHandle_, 100);
    if (err != ESP_OK) {
        return err;
    }

    // Send reset signal (>50μs low)
    err = sendLatchSignal();
    return err;
}

esp_err_t RGB::sendLatchSignal(void) {
    // WS2812 requires >50μs low signal as latch/reset
    // Use a simple delay which is more reliable than complex RMT reset transmission
    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms delay ensures proper latch (>50μs required)
    return ESP_OK;
}

esp_err_t RGB::initRmtChannel(rmt_channel_handle_t* channelHandle) {
    const uint32_t kFrequency10MHz = 10 * 1000000;

    rmt_tx_channel_config_t config = {
        .gpio_num = gpioNum_,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = kFrequency10MHz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .intr_priority = 1,
        .flags = {
            // TX channel config flags
            .invert_out = false,    // Output signal is not inverted
            .with_dma = false,      // DMA capability is not enabled
            .io_loop_back = false,  // No loopback
            .io_od_mode = false,    // GPIO is not open-drain
            .allow_pd =
                false  // Power domain is not allowed to be powered off in sleep mode
        }};

    return rmt_new_tx_channel(&config, channelHandle);
}

esp_err_t RGB::createCustomEncoder(rmt_encoder_handle_t* encoderHandle) {
    rmt_bytes_encoder_config_t encoderConfig = {
        .bit0 = bit0_, .bit1 = bit1_, .flags = {.msb_first = true}};

    return rmt_new_bytes_encoder(&encoderConfig, encoderHandle);
}
