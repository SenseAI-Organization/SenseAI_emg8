/*******************************************************************************
 * @file switch_sense.cpp
 * @brief Contains the definitions to work with the Switch class that
 * inherits from the Digital Sensor class.
 *
 * @version 0.2.1
 * @date 2024-10-04
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "switch_sense.hpp"

Switch::Switch(gpio_num_t pin, SwitchMode mode)
    : DigitalSensor(pin, switchModeToResistor(mode)), switchMode_(mode)  {
}

Switch::Switch(gpio_num_t pin, SwitchMode mode, bool externalResistor)
    : DigitalSensor(pin, switchModeToResistor(mode, externalResistor)), 
    switchMode_(mode) {}

Switch::~Switch() {}

esp_err_t Switch::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 128, "ID: %X, State: %d", getID(), pinState_);
    return ESP_OK;
}

uint8_t Switch::getID(void) const {
    return kSensorID_;
}

PinResistorMode Switch::switchModeToResistor(SwitchMode mode, 
                                                  bool externalResistor) {
    if (externalResistor) {
        return DISABLE_RESISTOR;
    }

    return mode == SwitchMode::kNormallyOpen ? PinResistorMode::PULL_DOWN
                                             : PinResistorMode::PULL_UP;
}

bool Switch::getState(void) const{
    return pinState_;
}

bool Switch::readState(void) {
    measure();
    return getState();
}

bool Switch::isPressed(bool invert) {
    measure();

    if (invert) {
        return !pinState_;
    }
    
    return (bool)pinState_;
}