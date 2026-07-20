/*****************************************************************************************
 * @file voltage_divider_sense.cpp
 * @brief Contains the definitions of the voltage divider's methods
 *
 * @version 0.2.0
 * @date 2025-08-01
 * @author Sense AI
 *****************************************************************************************
 ****************************************************************************************/

#include "voltage_divider_sense.hpp"

VoltageDivider::VoltageDivider(adc_channel_t channel, uint32_t r1, uint32_t r2)
    : AnalogSensor(channel), r1_(r1), r2_(r2) {
}

VoltageDivider::VoltageDivider(gpio_num_t pin, uint32_t r1, uint32_t r2)
    : AnalogSensor(pin), r1_(r1), r2_(r2) {
}

VoltageDivider::VoltageDivider(adc_channel_t channel, adc_atten_t attenuation,
                               adc_bitwidth_t resolution, uint32_t r1, uint32_t r2)
    : AnalogSensor(channel, attenuation, resolution), r1_(r1), r2_(r2) {
}

VoltageDivider::VoltageDivider(gpio_num_t pin, adc_atten_t attenuation,
                               adc_bitwidth_t resolution, uint32_t r1, uint32_t r2)
    : AnalogSensor(pin, attenuation, resolution), r1_(r1), r2_(r2) {
}

VoltageDivider::~VoltageDivider() {
}

esp_err_t VoltageDivider::measure(void) {
    esp_err_t err = AnalogSensor::measure();
    if (err != ESP_OK) {
        vOut_ = 0;
        vIn_ = 0;
        return err;
    }

    if (isCalibrationEnabled() && getCalibratedMv() > 0) {
        vOut_ = getCalibratedMv();
    } else {
        vOut_ = (static_cast<uint64_t>(getValue()) * getVRef()) / getAdcMaxValue();
    }

    if (r2_ > 0) {
        vIn_ = (static_cast<uint64_t>(vOut_) * (r1_ + r2_)) / r2_;
    } else {
        vIn_ = 0;  // Avoid division by zero
    }

    return ESP_OK;
}

esp_err_t VoltageDivider::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 128, "ID: 0x%X, Voltage: %u mV", getID(), getInputVoltage());
    return ESP_OK;
}

uint8_t VoltageDivider::getID(void) const {
    return kSensorID_;
}

uint16_t VoltageDivider::getInputVoltage(void) const {
    return vIn_;
}

uint16_t VoltageDivider::getOutputVoltage(void) const {
    return vOut_;
}

uint16_t VoltageDivider::readInputVoltage(void) {
    measure();
    return getInputVoltage();
}

uint16_t VoltageDivider::readOutputVoltage(void) {
    measure();
    return getOutputVoltage();
}

void VoltageDivider::setResistors(uint32_t r1, uint32_t r2) {
    r1_ = r1;
    r2_ = r2;
}

uint32_t VoltageDivider::getR1(void) const {
    return r1_;
}

uint32_t VoltageDivider::getR2(void) const {
    return r2_;
}

bool VoltageDivider::isVoltageSafe(uint16_t maxSafeVoltage) {
    return readInputVoltage() <= maxSafeVoltage;
}

/****************************************************************************************/
/*                                   Battery                                            */
/****************************************************************************************/

Battery::Battery(gpio_num_t pin)
    // Default to 1:1 divider and standard LiPo voltage range 3.2V-4.2V
    : VoltageDivider(pin, 10000, 10000), minV_(3200), maxV_(4200) {
}

Battery::Battery(gpio_num_t pin, uint16_t minVoltage, uint16_t maxVoltage)
    // Default to 1:1 divider
    : VoltageDivider(pin, 10000, 10000), minV_(minVoltage), maxV_(maxVoltage) {
}

Battery::Battery(gpio_num_t pin, uint32_t r1, uint32_t r2, uint16_t minVoltage,
                 uint16_t maxVoltage)
    : VoltageDivider(pin, r1, r2), minV_(minVoltage), maxV_(maxVoltage) {
}

esp_err_t Battery::measure(void) {
    // Call the parent measure() to get vIn_ calculated correctly
    esp_err_t err = VoltageDivider::measure();
    if (err != ESP_OK) {
        percentage_ = 0;
        return err;
    }

    if (maxV_ <= minV_) {  // Invalid voltage range configured
        percentage_ = 0;
        return ESP_ERR_INVALID_ARG;
    }

    if (vIn_ <= minV_) {
        percentage_ = 0;
    } else if (vIn_ >= maxV_) {
        percentage_ = 100;
    } else {
        percentage_ = static_cast<uint8_t>((static_cast<uint32_t>(vIn_ - minV_) * 100) /
                                           (maxV_ - minV_));
    }

    return ESP_OK;
}

esp_err_t Battery::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 128, "ID: 0x%X, Charge: %u%%, Voltage: %u mV", getID(),
             getChargePercentage(), getInputVoltage());
    return ESP_OK;
}

uint8_t Battery::getID(void) const {
    return kSensorID_;
}

uint8_t Battery::getChargePercentage(void) const {
    return percentage_;
}

uint8_t Battery::readChargePercentage(void) {
    measure();
    return getChargePercentage();
}

void Battery::setMaxVoltage(uint16_t voltage) {
    maxV_ = voltage;
}

uint16_t Battery::getMaxVoltage(void) const {
    return maxV_;
}

void Battery::setMinVoltage(uint16_t voltage) {
    minV_ = voltage;
}

uint16_t Battery::getMinVoltage(void) const {
    return minV_;
}
