#pragma once

// Diagnostic boot-only capture. No RMT receiver remains active in acquisition.
#include "driver/rmt_rx.h"

static bool IRAM_ATTR clockProbeDone(rmt_channel_handle_t,
                                    const rmt_rx_done_event_data_t* event,
                                    void* context) {
    BaseType_t woken = pdFALSE;
    size_t count = event->num_symbols;
    xQueueSendFromISR(static_cast<QueueHandle_t>(context), &count, &woken);
    return woken == pdTRUE;
}

static void probeBusClock(I2C& bus, gpio_num_t scl, unsigned busId) {
    rmt_channel_handle_t receiver = nullptr;
    rmt_rx_channel_config_t config = {};
    config.gpio_num = scl;
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = 20000000;  // durations below are in 50 ns ticks
    config.mem_block_symbols = 64;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&config, &receiver));
    // RX creation enables the internal pull-up. Restore the existing I2C
    // setting immediately; leave its output matrix and open-drain mode intact.
    ESP_ERROR_CHECK(gpio_pullup_dis(scl));
    QueueHandle_t done = xQueueCreate(1, sizeof(size_t));
    if (!done) ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    rmt_rx_event_callbacks_t callbacks = {};
    callbacks.on_recv_done = clockProbeDone;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(receiver, &callbacks, done));
    ESP_ERROR_CHECK(rmt_enable(receiver));
    rmt_receive_config_t capture = {};
    capture.signal_range_min_ns = 100;
    capture.signal_range_max_ns = 100000;

    for (unsigned trial = 0; trial < 3; ++trial) {
        for (unsigned read = 0; read < 2; ++read) {
            rmt_symbol_word_t symbols[64] = {};
            vTaskDelay(pdMS_TO_TICKS(2));
            ESP_ERROR_CHECK(rmt_receive(receiver, symbols, sizeof(symbols), &capture));
            uint8_t bytes[2] = {0xC3, 0xC3}; // single-shot, comparator disabled
            int64_t begin = esp_timer_get_time();
            esp_err_t err = read ? bus.read(0x48, 0, bytes, 2)
                                 : bus.write(0x48, 1, bytes, 2);
            int64_t elapsed = esp_timer_get_time() - begin;
            size_t count = 0;
            bool received = xQueueReceive(done, &count, pdMS_TO_TICKS(100)) == pdTRUE;
            printf("#SCLPROBE:%u,%u,%u,%d,%lld,%u,%u", busId, trial, read,
                   (int)err, (long long)elapsed, received, (unsigned)count);
            for (size_t i = 0; i < count && i < 64; ++i)
                printf(",%u:%u:%u:%u", symbols[i].level0, symbols[i].duration0,
                       symbols[i].level1, symbols[i].duration1);
            printf("\n");
            ESP_ERROR_CHECK(err);
            if (!received) ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
        }
    }
    ESP_ERROR_CHECK(rmt_disable(receiver));
    ESP_ERROR_CHECK(rmt_del_channel(receiver));
    vQueueDelete(done);
}
