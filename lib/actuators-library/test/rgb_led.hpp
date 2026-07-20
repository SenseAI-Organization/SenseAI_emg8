/**
 * led.hpp file
 * Sense Ai SAS
 */
#pragma once

#include <stdint.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

class RGB_LED {
public:
    // Constructor
    RGB_LED();

    // Method to initialize the LED (RMT)
    void init(void);

    // Method to set the color of the LED
    void setColor(uint8_t red, uint8_t green, uint8_t blue);

    // Method to turn the LED off
    void turnOff(void);

    // Method to check if the LED is on or off
    bool getStatus(void) const;

    // Method to retrieve the current color
    void getColor(uint8_t& red, uint8_t& green, uint8_t& blue) const;

private:
    // T0H, Level, T0L, Level
    // const rmt_symbol_word_t bit0 = {1, 3, 0, 8};
    const rmt_symbol_word_t bit0 = {3, 1, 8, 0};

    // Mark, T1H, Space, T1L
    // const rmt_symbol_word_t bit1 = {1, 7, 0, 3};
    const rmt_symbol_word_t bit1 = {7, 1, 3, 0};

    // Low level, T, X,X noot needed, here for safety
    // const rmt_symbol_word_t latch_signal = {0, 2000, 0, 0};
    const rmt_symbol_word_t latch_signal = {2000, 0, 0, 0};

    // Private method to simulate the hardware update process
    void updateHardware(rmt_channel_handle_t channel_handle,
                        rmt_encoder_handle_t encoder_handle);
    esp_err_t init_rmt_channel(rmt_channel_handle_t* channel_handle);
    esp_err_t create_custom_encoder(rmt_encoder_handle_t* encoder_handle);
    esp_err_t send_latch_signal(rmt_channel_handle_t tx_channel);
    esp_err_t sensor_burst_read(uint8_t reg_addr, uint8_t* data, size_t len);

    // Member variables to store the color intensity for each component
    uint8_t redIntensity;
    uint8_t greenIntensity;
    uint8_t blueIntensity;
    bool isOn;

    rmt_channel_handle_t channel_handle = NULL;
    rmt_encoder_handle_t encoder_handle = NULL;
};
