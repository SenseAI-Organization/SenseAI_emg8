/*******************************************************************************
 * @file DYP_A22YYMW.cpp
 * @brief Contains the definitions of the DYP_A22YYMW class methods.
 *
 * This sensor has 2 signals: one can generate a trigger and the other receives
 * the echo by different pins.
 *
 * @version v0.1.0
 * @date 2025-15-05
 * @author Isabella Garcia isa@sense-ai.co
 *******************************************************************************/

#include "DYP_A22YYMW.hpp"
#include "sensors_sense.hpp"

DYP_A22YYMW::DYP_A22YYMW(gpio_num_t trigPin, gpio_num_t echoPin)
    : trigPin_(trigPin), echoPin_(echoPin) {
}

DYP_A22YYMW::~DYP_A22YYMW() {
}

esp_err_t DYP_A22YYMW::init(void) {
    esp_err_t err = gpio_set_direction(trigPin_, GPIO_MODE_OUTPUT);

    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    err = gpio_set_direction(echoPin_, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    err = gpio_set_level(trigPin_, 1);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void DYP_A22YYMW::sendPulse() const {
    const uint8_t kHigh = 1;
    const uint8_t kLow = 0;

    esp_err_t err = gpio_set_level(trigPin_, kLow);
    if (err != ESP_OK) return;

    esp_rom_delay_us(10);

    err = gpio_set_level(trigPin_, kHigh);
    if (err != ESP_OK) return;
}

esp_err_t DYP_A22YYMW::readDistance(float* distance) const {
    if (distance == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    sendPulse();

    // wait till echo turns high
    int64_t startTime = esp_timer_get_time();
    uint32_t timeOut = 14000;
    const uint8_t kHigh = 1;
    const uint8_t kLow = 0;

    // vTaskSuspendAll();

    while (gpio_get_level(echoPin_) == kLow) {
        if ((esp_timer_get_time() - startTime) > timeOut) {  // Timeout
            return ESP_ERR_TIMEOUT;
        }
    }

    // Measure the lenght
    int64_t pulseStart = esp_timer_get_time();
    timeOut = 28000;  // 30ms max.

    while (gpio_get_level(echoPin_) == kHigh) {
        if ((esp_timer_get_time() - pulseStart) > timeOut) {
            *distance = 4000;

            return ESP_ERR_TIMEOUT;
        }
    }

    int64_t pulseEnd = esp_timer_get_time();

    // xTaskResumeAll();

    int64_t duration = pulseEnd - pulseStart;

    // Calculate distance
    constexpr float kFactor = 0.1716;  // (0.03432 / 2.0) V at 20°C
    *distance = duration * kFactor;

    return ESP_OK;
}

esp_err_t DYP_A22YYMW::measure(void) {
    esp_err_t err = readDistance(&distance_);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t DYP_A22YYMW::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    float distance = 0;
    esp_err_t err = readDistance(&distance);
    if (err != ESP_OK) {
        snprintf(_buff, 128, "Error reading distance: %d", err);

        return err;
    }

    snprintf(_buff, 128, "Distance: %.2f mm", distance);

    return ESP_OK;
}

uint8_t DYP_A22YYMW::getID(void) const {
    return 0x21;
}
