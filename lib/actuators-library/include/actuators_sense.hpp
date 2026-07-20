/*******************************************************************************
 * @file actuators_sense.hpp
 * @brief Contains the declarations to work with Actuator Interface and some
 * generic abstract actuators for ESP32 SoCs as part of the Sense Ecosystem.
 *
 * @version 0.1.2
 * @date 2025-10-1
 * @author emmanuel@sense-ai.co, mateor@sense-ai.co
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/******************************************************************************/
/*                                Actuator                                    */
/******************************************************************************/

/**
 * @class Actuator
 *
 * @brief Abstract base class defining the interface for all actuator devices
 */
class Actuator {
public:
    /**
     * @brief Virtual destructor for the Actuator base class
     */
    virtual ~Actuator();

    /**
     * @brief Initialize the actuator with its specific configuration
     * @return ESP_OK Success - ESP_FAIL Parameter error or initialization failure
     */
    virtual esp_err_t init(void) = 0;

    /**
     * @brief Turn on the actuator
     * @return ESP_OK Success - ESP_FAIL Parameter error or operation failure
     */
    virtual esp_err_t turnOn(void) = 0;

    /**
     * @brief Turn off the actuator
     * @return ESP_OK Success - ESP_FAIL Parameter error or operation failure
     */
    virtual esp_err_t turnOff(void) = 0;

    /**
     * @brief Generate a pulse signal for the specified duration
     * @param millisOn Duration in milliseconds for the pulse to remain on
     * @return ESP_OK Success - ESP_FAIL Parameter error or operation failure
     */
    virtual esp_err_t pulse(uint32_t millisOn) = 0;

    /**
     * @brief Generate a cycle signal (on-off pattern) for the specified duration
     * @param millisOn Duration in milliseconds for each on/off phase
     * @return ESP_OK Success - ESP_FAIL Parameter error or operation failure
     */
    virtual esp_err_t cycle(uint32_t millisOn) = 0;
};

/******************************************************************************/
/*                                LED                                         */
/******************************************************************************/

/**
 * @class LED
 *
 * @brief A class to manage basic LED actuators using GPIO pins
 */
class LED : public Actuator {
public:
    /**
     * @enum LedState
     * @brief Enumeration for LED state representation
     */
    enum class LedState {
        kOff = 0,  ///< LED is turned off (low level)
        kOn = 1    ///< LED is turned on (high level)
    };

    /**
     * @brief Construct a new LED object
     *
     * @param pin GPIO pin number to control the LED
     */
    LED(gpio_num_t pin);

    /**
     * @brief Destructor for the LED object
     */
    ~LED();

    /**
     * @brief Initialize the LED GPIO pin with output configuration
     * @return ESP_OK Success - ESP_ERR_INVALID_ARG Invalid GPIO pin
     */
    esp_err_t init(void) override;

    /**
     * @brief Turn on the LED by setting GPIO pin to high level
     * @return ESP_OK Success - ESP_FAIL GPIO operation failure
     */
    esp_err_t turnOn(void) override;

    /**
     * @brief Turn off the LED by setting GPIO pin to low level
     * @return ESP_OK Success - ESP_FAIL GPIO operation failure
     */
    esp_err_t turnOff(void) override;

    /**
     * @brief Generate a pulse signal by turning LED on for specified duration
     * @param millisOn Duration in milliseconds to keep the LED on
     * @return ESP_OK Success - ESP_FAIL GPIO operation failure
     */
    esp_err_t pulse(uint32_t millisOn) override;

    /**
     * @brief Generate a cycle signal (placeholder implementation)
     * @param millisOn Duration in milliseconds for the cycle
     * @return ESP_OK Success - ESP_FAIL Operation failure
     *
     * @note Currently returns ESP_OK without performing any operation
     */
    esp_err_t cycle(uint32_t millisOn) override;

private:
    gpio_num_t pin_;  ///< GPIO pin number assigned to the LED
    LedState state_;  ///< Current state of the LED
};

/******************************************************************************/
/*                                RGB                                         */
/******************************************************************************/

/**
 * @class RGB
 *
 * @brief A class to manage RGB LED actuators using WS2812 protocol with RMT peripheral
 */
class RGB : public Actuator {
public:
    /**
     * @brief Construct a new RGB object with default color (off)
     *
     * @param gpioNum GPIO pin number to control the RGB LED
     */
    RGB(gpio_num_t gpioNum);

    /**
     * @brief Construct a new RGB object with initial color values
     *
     * @param gpioNum GPIO pin number to control the RGB LED
     * @param red Initial red intensity value (0-255)
     * @param green Initial green intensity value (0-255)
     * @param blue Initial blue intensity value (0-255)
     */
    RGB(gpio_num_t gpioNum, uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief Destructor for the RGB object
     *
     * @note Automatically cleans up RMT resources and turns off the LED
     */
    ~RGB();

    /**
     * @brief Initialize the RGB LED with RMT channel and encoder configuration
     * @return ESP_OK Success - ESP_ERR_INVALID_ARG Invalid GPIO pin -
     * ESP_ERR_INVALID_STATE RMT initialization failure
     */
    esp_err_t init(void) override;

    /**
     * @brief Turn on the RGB LED with current color settings
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware update failure
     */
    esp_err_t turnOn(void) override;

    /**
     * @brief Turn off the RGB LED by setting all colors to 0
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware update failure
     */
    esp_err_t turnOff(void) override;

    /**
     * @brief Generate a pulse signal by turning RGB LED on for specified duration
     * @param millisOn Duration in milliseconds to keep the LED on
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware operation failure
     */
    esp_err_t pulse(uint32_t millisOn) override;

    /**
     * @brief Generate an on-off cycle with the specified duration for each phase
     * @param millisOn Duration in milliseconds for each on/off phase
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware operation failure
     */
    esp_err_t cycle(uint32_t millisOn) override;

    /**
     * @brief Set the RGB color values
     * @param red Red intensity value (0-255)
     * @param green Green intensity value (0-255)
     * @param blue Blue intensity value (0-255)
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware update failure
     */
    esp_err_t setColor(uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief Get the current RGB color values
     * @param red Reference to store current red intensity value
     * @param green Reference to store current green intensity value
     * @param blue Reference to store current blue intensity value
     */
    void getColor(uint8_t& red, uint8_t& green, uint8_t& blue) const;

    /**
     * @brief Check if the RGB LED is currently on
     * @return true if the LED is on, false if it's off
     */
    bool isOn(void) const;

private:
    // WS2812 timing at 10MHz: bit0 = 0.4μs high + 0.85μs low, bit1 = 0.8μs high + 0.45μs
    // low Alternative timing - try different values for better compatibility
    const rmt_symbol_word_t bit0_ = {3, 1, 9,
                                     0};  ///< WS2812 bit 0 timing: 0.3μs high, 0.9μs low
    const rmt_symbol_word_t bit1_ = {9, 1, 3,
                                     0};  ///< WS2812 bit 1 timing: 0.9μs high, 0.3μs low

    gpio_num_t gpioNum_;  ///< GPIO pin number assigned to the RGB LED
    rmt_channel_handle_t channelHandle_ =
        NULL;  ///< RMT channel handle for WS2812 communication
    rmt_encoder_handle_t encoderHandle_ = NULL;  ///< RMT encoder handle for data encoding

    uint8_t redIntensity_;    ///< Current red color intensity (0-255)
    uint8_t greenIntensity_;  ///< Current green color intensity (0-255)
    uint8_t blueIntensity_;   ///< Current blue color intensity (0-255)

    bool isOn_;  ///< Current on/off state of the RGB LED

    /**
     * @brief Update hardware with current color values if LED is on, otherwise turn off
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware update failure
     */
    esp_err_t updateHardware(void);

    /**
     * @brief Update hardware with specific RGB color values
     * @param red Red intensity value (0-255)
     * @param green Green intensity value (0-255)
     * @param blue Blue intensity value (0-255)
     * @return ESP_OK Success - ESP_ERR_INVALID_STATE RMT not initialized - ESP_FAIL
     * Hardware update failure
     */
    esp_err_t updateHardware(uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief Send latch signal required by WS2812 protocol (>50μs low)
     * @return ESP_OK Success
     */
    esp_err_t sendLatchSignal(void);

    /**
     * @brief Initialize RMT channel for WS2812 communication
     * @param channelHandle Pointer to store the created channel handle
     * @return ESP_OK Success - ESP_FAIL RMT channel creation failure
     */
    esp_err_t initRmtChannel(rmt_channel_handle_t* channelHandle);

    /**
     * @brief Create custom byte encoder for WS2812 protocol
     * @param encoderHandle Pointer to store the created encoder handle
     * @return ESP_OK Success - ESP_FAIL Encoder creation failure
     */
    esp_err_t createCustomEncoder(rmt_encoder_handle_t* encoderHandle);
};
