/*******************************************************************************
 * @file switch_sense.hpp
 * @brief Contains the declarations to work with the Switch class that
 * inherits from the Digital Sensor class.
 *
 * @version 0.2.1
 * @date 2024-10-04
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <stdio.h>

#include "driver/gpio.h"

#include "sensors_sense.hpp"

/**
 * @brief Represents a switch that extends the DigitalSensor class.
 *
 * The Switch class provides functionality to manage a switch
 * sensor, allowing configuration as either normally open or normally closed.
 */
class Switch : public DigitalSensor {
public:

    /**
     * @enum SwitchMode
     * @brief Enumeration to represent the electrical switch modes.
     */
    enum class SwitchMode {
        kNormallyOpen   = 0,    ///< Open circuit when inactive.
        kNormallyClosed = 1     ///< Closed circuit when inactive.
    };

    /**
     * @brief Constructor for Switch with specified pin and mode.
     * @param pin GPIO pin number for the switch.
     * @param mode Mode (open or closed) of the switch.
     */
    Switch(gpio_num_t pin, SwitchMode mode);

    /**
     * @brief Constructor for Switch with specified pin and mode.
     * @param pin GPIO pin number for the switch.
     * @param mode Mode (open or closed) of the switch.
     * @param externalResistor Use external resistor (true) or not (false).
     */
    Switch(gpio_num_t pin, SwitchMode mode, bool externalResistor);

    /**
     * @brief Destructor for Switch.
     */
    virtual ~Switch();

    /**
     * @brief Creates a report with the state and ID of the sensor.
     */
    esp_err_t getReport(char* _buff) const override;

    /**
     * @brief Returns the ID of the Switch class.
     */
    uint8_t getID(void) const override;

    /**
     * @brief Returns the last measured state of the Switch.
     * @param state Reference to allocate the current state of the sensor.
     * @return ESP_OK on success, error code otherwise.
     * @note This should be used knowing the configuration of the 
     * internal/external resistor and the switch mode of the Switch.
     */
    bool getState(void) const;

    /**
     * @brief Reads the current state of the switch.
     * @return current state of the sensor.
     * @note This should be used knowing the configuration of the 
     * internal/external resistor and the switch mode of the Switch.
     */
    bool readState(void);

    /**
     * @brief Checks if the switch is pressed.
     * @param invert False by default, this inverts the return values.
     * @return True if switch is active (pressed), false otherwise.
     * @note This method should be used with invert =  true if there's an
     * external resistor and the convention is not being followed:
     * Pull-up for normally closed and Pull-down for normally open.
     */
    bool isPressed(bool invert = false);

private:
    const uint8_t kSensorID_ = 0x01;
    SwitchMode switchMode_;

    static PinResistorMode switchModeToResistor(SwitchMode mode, 
                                                bool externalResistor = false);
};