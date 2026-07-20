#include "rgb_led.hpp"

// #ifdef __cplusplus
// extern "C" {
// #endif

// Constructor
RGB_LED::RGB_LED() : redIntensity(0), greenIntensity(0), blueIntensity(0), isOn(false) {
}

// Method to set the color of the LED
void RGB_LED::setColor(uint8_t red, uint8_t green, uint8_t blue) {
    redIntensity = red;
    greenIntensity = green;
    blueIntensity = blue;
    isOn = true;
    updateHardware(channel_handle, encoder_handle);
}

// Method to turn the LED off
void RGB_LED::turnOff(void) {
    redIntensity = 0;
    greenIntensity = 0;
    blueIntensity = 0;
    isOn = false;
    updateHardware(channel_handle, encoder_handle);
}

// Method to check if the LED is on or off
bool RGB_LED::getStatus(void) const {
    return isOn;
}

// Method to retrieve the current color
void RGB_LED::getColor(uint8_t& red, uint8_t& green, uint8_t& blue) const {
    red = redIntensity;
    green = greenIntensity;
    blue = blueIntensity;
}

void RGB_LED::updateHardware(rmt_channel_handle_t channel_handle,
                             rmt_encoder_handle_t encoder_handle) {
    uint8_t color_data[3] = {greenIntensity, redIntensity,
                             blueIntensity};  // The order is GRB
    rmt_transmit_config_t tx_config = {.loop_count = 0, .flags = {.eot_level = 0}};

    size_t bytes_encoded = 0;
    rmt_encode_state_t encode_state = RMT_ENCODING_RESET;
    bytes_encoded = encoder_handle->encode(encoder_handle, channel_handle, color_data,
                                           sizeof(color_data), &encode_state);
    (void)bytes_encoded;

    // Send the encoded data
    rmt_transmit(channel_handle, encoder_handle, color_data, sizeof(color_data),
                 &tx_config);
    // rmt_tx_wait_all_done(channel_handle, 100); // Puede sobrar. Probando TODO
    send_latch_signal(channel_handle);
    rmt_tx_wait_all_done(channel_handle, 100);
}

void RGB_LED::init(void) {
    ESP_ERROR_CHECK(init_rmt_channel(&channel_handle));
    ESP_ERROR_CHECK(rmt_enable(channel_handle));

    ESP_ERROR_CHECK(create_custom_encoder(&encoder_handle));
}

// Function to initialize RMT
esp_err_t RGB_LED::init_rmt_channel(rmt_channel_handle_t* channel_handle) {
    const uint32_t kFrequency_10MHz = 10 * 1000000;

    rmt_tx_channel_config_t config = {// GPIO_NUM_48, for DevKit.
                                      .gpio_num = GPIO_NUM_2,
                                      .clk_src = RMT_CLK_SRC_DEFAULT,
                                      .resolution_hz = kFrequency_10MHz,
                                      .mem_block_symbols = 64,
                                      .trans_queue_depth = 1,
                                      .flags = {.invert_out = false,
                                                .with_dma = false,
                                                .io_loop_back = false,
                                                .io_od_mode = false},
                                      .intr_priority = 1};

    return rmt_new_tx_channel(&config, channel_handle);
}

// Function to create a bytes encoder with custom bit definitions
esp_err_t RGB_LED::create_custom_encoder(rmt_encoder_handle_t* encoder_handle) {
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = bit0, .bit1 = bit1, .flags = {.msb_first = true}};

    return rmt_new_bytes_encoder(&encoder_config, encoder_handle);
}

esp_err_t RGB_LED::send_latch_signal(rmt_channel_handle_t tx_channel) {
    const uint8_t latch_payload[1] = {0};  // Example payload, contents depend on encoder
    size_t latch_payload_size = sizeof(latch_payload);
    rmt_transmit_config_t latch_config = {.loop_count = 0, .flags = {.eot_level = 0}};

    size_t bytes_encoded = 0;
    rmt_encode_state_t encode_state = RMT_ENCODING_RESET;
    bytes_encoded = encoder_handle->encode(encoder_handle, channel_handle, latch_payload,
                                           latch_payload_size, &encode_state);
    (void)bytes_encoded;
    return rmt_transmit(tx_channel, encoder_handle, latch_payload, latch_payload_size,
                        &latch_config);
}

// #ifdef __cplusplus
// }
// #endif
