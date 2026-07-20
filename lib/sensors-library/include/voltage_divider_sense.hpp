/*******************************************************************************
 * @file voltage_divider_sense.hpp
 * @brief Contains the declarations of the voltage divider's methods
 * 
 * @version 0.1.2
 * @date 2024-06-21
 * @author Sense AI
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <stdio.h>

#include "driver/gpio.h"
#include "sensors_sense.hpp"

/**
 * @brief Represents a voltage divider sensor that extends the AnalogSensor
 *        class.
 *
 * The VoltageDivider class provides functionality to manage a voltage divider,
 * allowing calculation of the input voltage based on the output voltage and
 * the known resistor values. It leverages the underlying AnalogSensor's
 * calibrated voltage reading for accuracy.
 */
class VoltageDivider : public AnalogSensor {
public:
    /**
     * @brief Constructor for VoltageDivider with specified channel and resistor
     *        values.
     * @param channel ADC channel connected to the output of the voltage divider.
     * @param r1 Resistor value closer to the input voltage in ohms.
     * @param r2 Resistor value closer to ground in ohms.
     */
    VoltageDivider(adc_channel_t channel, uint32_t r1, uint32_t r2);

    /**
     * @brief Initializes using a GPIO pin.
     * @param pin GPIO pin connected to the voltage divider output.
     * @param r1 Resistor value closer to the input voltage (ohms).
     * @param r2 Resistor value closer to ground (ohms).
     */
    VoltageDivider(gpio_num_t pin, uint32_t r1, uint32_t r2);

    /**
     * @brief Initializes using ADC channel with detailed settings.
     * @param channel ADC channel connected to the voltage divider output.
     * @param attenuation Attenuation setting for the ADC.
     * @param resolution Bit resolution of the ADC conversion.
     * @param r1 Resistor value closer to the input voltage (ohms).
     * @param r2 Resistor value closer to ground (ohms).
     */
    VoltageDivider(adc_channel_t channel, adc_atten_t attenuation,
                   adc_bitwidth_t resolution, uint32_t r1, uint32_t r2);

    /**
     * @brief Initializes using GPIO pin with detailed settings.
     * @param pin GPIO pin connected to the voltage divider output.
     * @param attenuation Attenuation setting for the ADC.
     * @param resolution Bit resolution of the ADC conversion.
     * @param r1 Resistor value closer to the input voltage (ohms).
     * @param r2 Resistor value closer to ground (ohms).
     */
    VoltageDivider(gpio_num_t pin, adc_atten_t attenuation,
                   adc_bitwidth_t resolution, uint32_t r1, uint32_t r2);

    /**
     * @brief Destructor for VoltageDivider.
     */
    virtual ~VoltageDivider();

    /**
     * @brief Measures the voltage and calculates the input and output voltages
     *        of the divider.
     *
     * This method calls the base AnalogSensor's measure() to get a calibrated
     * reading of the divider's output voltage, then calculates the
     * corresponding input voltage.
     *
     * @return esp_err_t ESP_OK on success, error code on failure.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Generates the report for the input voltage of the Voltage Divider.
     * @param _buff Buffer where the string report is going to be placed.
     * @return ESP_OK on success, error code on failure.
     */
    esp_err_t getReport(char* _buff) const override;

    /**
     * @brief Returns the ID of the Voltage Divider (0x10)
     */
    uint8_t getID(void) const override;

    /**
     * @brief Returns the calculated input voltage of the voltage divider in mV.
     */
    uint16_t getInputVoltage(void) const;

    /**
     * @brief Returns the measured output voltage of the voltage divider in mV.
     */
    uint16_t getOutputVoltage(void) const;

    /**
     * @brief Performs a measurement and returns the calculated input voltage in mV.
     */
    uint16_t readInputVoltage(void);

    /**
     * @brief Performs a measurement and returns the measured output voltage in mV.
     */
    uint16_t readOutputVoltage(void);

    /**
     * @brief Sets the resistor values for the voltage divider.
     * @param r1 Resistor closer to the input voltage in ohms.
     * @param r2 Resistor closer to the ground in ohms.
     */
    void setResistors(uint32_t r1, uint32_t r2);

    /**
     * @brief Gets the value of the resistor closer to the input voltage.
     * @return Resistor value in ohms.
     */
    uint32_t getR1(void) const;

    /**
     * @brief Gets the value of the resistor closer to the ground.
     * @return Resistor value in ohms.
     */
    uint32_t getR2(void) const;

    /**
     * @brief Checks if the current voltage is within safe operational limits.
     * @param safeVoltage Maximum safe operating voltage in mV.
     * @return true if voltage is safe, false otherwise.
     */
    bool isVoltageSafe(uint16_t safeVoltage);

protected:
    uint32_t r1_ = 0;
    uint32_t r2_ = 0;

    uint16_t vOut_ = 0; // Output voltage in mV
    uint16_t vIn_ = 0;  // Input voltage in mV

private:
    const uint8_t kSensorID_ = 0x10;
};


/**
 * @brief Represents a battery sensor, extending VoltageDivider to calculate
 *        charge percentage.
 */
class Battery : public VoltageDivider {
public:
    /**
     * @brief Constructs a Battery sensor using a GPIO pin.
     * Uses a default 1:1 voltage divider (R1=10k, R2=10k) and default
     * LiPo voltage range (3200mV-4200mV).
     * @param pin GPIO pin connected to the voltage divider output.
     */
    Battery(gpio_num_t pin);

    /**
     * @brief Constructs a Battery sensor with custom voltage range.
     * Uses a default 1:1 voltage divider (R1=10k, R2=10k).
     * @param pin GPIO pin connected to the voltage divider output.
     * @param minVoltage Minimum battery voltage for 0% charge (in mV).
     * @param maxVoltage Maximum battery voltage for 100% charge (in mV).
     */
    Battery(gpio_num_t pin, uint16_t minVoltage, uint16_t maxVoltage);

    /**
     * @brief Constructs a Battery sensor with custom resistors and voltage range.
     * @param pin GPIO pin connected to the voltage divider output.
     * @param r1 Resistor value closer to the battery positive terminal (ohms).
     * @param r2 Resistor value closer to ground (ohms).
     * @param minVoltage Minimum battery voltage for 0% charge (in mV).
     * @param maxVoltage Maximum battery voltage for 100% charge (in mV).
     */
    Battery(gpio_num_t pin, uint32_t r1, uint32_t r2, uint16_t minVoltage,
            uint16_t maxVoltage);

    /**
     * @brief Measures the battery voltage and calculates the charge percentage.
     * @return esp_err_t ESP_OK on success, error code on failure.
     */
    esp_err_t measure(void) override;
    
    /**
     * @brief Generates a report with the battery's charge percentage.
     * @param _buff Buffer where the string report is going to be placed.
     * @return ESP_OK on success, error code on failure.
     */
    esp_err_t getReport(char* _buff) const override;
    
    /**
     * @brief Returns the ID of the Battery sensor (0x11).
     */
    uint8_t getID(void) const override;

    /**
     * @brief Returns the last calculated charge percentage.
     */
    uint8_t getChargePercentage(void) const;

    /**
     * @brief Performs a measurement and returns the new charge percentage.
     */
    uint8_t readChargePercentage(void);

    void setMaxVoltage(uint16_t voltage);
    uint16_t getMaxVoltage(void) const;
    void setMinVoltage(uint16_t voltage);
    uint16_t getMinVoltage(void) const;

private:
    const uint8_t kSensorID_ = 0x11;
    uint8_t percentage_ = 0;
    uint16_t minV_ = 0;
    uint16_t maxV_ = 0;
};