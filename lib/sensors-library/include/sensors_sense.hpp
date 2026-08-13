/*******************************************************************************
 * @file sensors_sense.hpp
 * @brief Contains the declarations to work with Interface Sensor, and the
 * Analog Sensor and Digital Sensor abstract classes.
 *
 * @version 0.3.0
 * @date 2025-08-01
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <stdio.h>

#include <map>
#include <tuple>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ADC_CHANNEL_UNDEFINED -1

/**
 * @enum
 * @brief Internal resistor configuration (mode)
 */
typedef enum { DISABLE_RESISTOR, PULL_UP, PULL_DOWN } PinResistorMode;

constexpr int kAdcValueError = -99999;

/******************************************************************************/
/*                                 Sensor                                     */
/******************************************************************************/

/**
 * @brief Base class for all sensors, providing a common interface for
 * configuration, initialization, and data reading.
 *
 * This abstract class defines the basic operations that all sensor types must
 * support, including configuration, initialization, and the mechanism to read
 * data from the sensor. Derived classes must provide concrete implementations
 * of the pure virtual methods to handle hardware-specific details.
 */
class Sensor {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     */
    virtual ~Sensor();

    /**
     * @brief Initializes the sensor.
     *
     * Must be implemented by derived classes to perform any necessary
     * initialization before the sensor can start operating. This could include
     * setting up GPIO pins, configuring communication protocols, or calibrating
     * the sensor.
     *
     * @return esp_err_t Returns ESP_OK on success or error code on failure.
     */
    virtual esp_err_t init(void) = 0;

    /**
     * @brief Realizes a measure of the sensor.
     *
     * Derived classes must implement the logic to read the sensor.
     * @return esp_err_t Returns ESP_OK on success or an appropriate error code
     * on failure.
     */
    virtual esp_err_t measure(void) = 0;

    /**
     * @brief Organizes the data of the sensor on a buffer.
     * @param _buff Buffer where the string is going to be stored. It's recommended
     * to use an array for safety reasons.
     * @return esp_err_t Returns ESP_OK on success or an appropriate error code
     * on failure.
     */
    virtual esp_err_t getReport(char* _buff) const = 0;

    /**
     * @brief Function to get the ID of the sensor.
     * @return uint8_t Sensor ID (Every end level sensor should have one).
     * 0x00 by default.
     */
    virtual uint8_t getID(void) const = 0;

protected:
    uint8_t sensorID_ = 0x00;
};

/******************************************************************************/
/*                                 Digital Sensor                             */
/******************************************************************************/

/**
 * @brief Represents a digital sensor, extending the generic Sensor interface
 * with digital-specific functionalities.
 *
 * This class provides mechanisms to configure and interact with a digital
 * sensor, including setting and retrieving the GPIO pin configuration,
 * pin number, pin mode, internal pull-up/pull-down resistors, and handling
 * pin interrupts.
 */
class DigitalSensor : public Sensor {
public:
    typedef void (*DigitalSensorCallback)(void*);

    /**
     * @brief Constructs a new DigitalSensor object with default settings.
     */
    DigitalSensor(gpio_num_t pinNumber, PinResistorMode resistor);

    /**
     * @brief Destroys the DigitalSensor object. Changes pin to default.
     */
    virtual ~DigitalSensor();  // Destructor

    /**
     * @brief Initializes the sensor.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t init(void) override;

    /**
     * @brief Performs a reading of the sensor.
     * @return ESP_OK on success or error code on failure.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Gets the current GPIO config.
     * @return Current GPIO configuration.
     */
    gpio_config_t getPinConfiguration(void) const;

    /**
     * @brief Gets the GPIO pin number.
     * @return GPIO pin number.
     */
    gpio_num_t getPinNumber(void) const;

    /**
     * @brief Sets the GPIO pin number.
     * @param pinNumber GPIO pin number.
     */
    esp_err_t setPinNumber(gpio_num_t pinNumber);

    /**
     * @brief Sets internal resistor mode for pin.
     * @param resistorConfig Resistor configuration.
     */
    esp_err_t setPinInternalResistor(PinResistorMode resistorConfig);

    /**
     * @brief Gets internal resistor config.
     * @return Resistor configuration.
     */
    PinResistorMode getPinInternalResistor(void) const;

    /**
     * @brief Sets the pin interrupt type.
     * @param interruptType Interrupt type.
     */
    esp_err_t setPinInterrupt(gpio_int_type_t interruptType);

    /**
     * @brief Gets the current pin state.
     * @return Pin state, -1 on failure.
     */
    int8_t getPinState(void) const;

    /**
     * @brief Gets the previous pin state.
     * @return Previous pin state. -2 on failure.
     */
    int8_t getPreviousPinState(void) const;

    /**
     * @brief Configures the GPIO interrupt for the sensor.
     *
     * Sets up an interrupt for the sensor's GPIO pin based on the specified
     * type. It also registers an ISR handler and initializes a semaphore for
     * synchronization between the ISR and the sensor handling task.
     *
     * @param intrType The type of GPIO interrupt (e.g., rising edge).
     * @param callback The function to call when the interrupt occurs.
     * @param arg Additional argument passed to the callback function.
     *
     * @return ESP_OK if successful, or esp_err_t code indicating the error.
     */
    esp_err_t configureInterrupt(gpio_int_type_t intrType, DigitalSensorCallback callback,
                                 void* arg);

    /**
     * @brief Starts a FreeRTOS task for handling sensor actuation.
     * @param taskName Pointer to a char array for the task's name, useful for
     *                 debugging and monitoring.
     * @param taskPriority The task's priority level. Higher priority tasks
     *                 preempt lower priority ones as per FreeRTOS scheduling.
     * @param stackSize Maximum size in bytes to the task to be used.
     * @note `handleSwitchActuation` should be efficient and manage all sensor
     *       states to avoid task overruns or missed events.
     */
    void startHandlerTask(const char* taskName, UBaseType_t taskPriority, int stackSize);

protected:
    /**
     * @brief Interrupt Service Routine (ISR) for the sensor.
     *
     * This ISR is triggered by a GPIO interrupt. It gives a semaphore to
     * notify the task handling the sensor actuation.
     *
     * @param arg Pointer to the DigitalSensor instance that the ISR belongs to.
     */
    static IRAM_ATTR void isr_handler(void* arg);  // Private ISR handler

    /**
     * @brief Handles sensor actuation in a dedicated task.
     *
     * Continuously waits for a semaphore to be given, indicating an interrupt
     * has occurred. Upon receiving the semaphore, it executes a callback if
     * one is registered.
     */
    void handleSwitchActuation(void);

    uint64_t lastInterruptTime_;

    gpio_num_t pinNumber_;

    gpio_config_t sensorConfig_ = {0,  // Default configuration
                                   GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE,
                                   GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};

    int8_t pinState_ = -1;       // To indicate it hasn't read anything yet
    int8_t previousState_ = -2;  // To identify if it's the first sample

    SemaphoreHandle_t semaphore_;

    DigitalSensorCallback callback_ = nullptr;
    void* callbackArg_ = nullptr;
};

/******************************************************************************/
/*                                  Analog Sensor                             */
/******************************************************************************/

/**
 * @brief Represents an analog sensor, extending the generic Sensor interface
 * with analog-specific functionalities.
 *
 * This class provides mechanisms to configure and interact with an analog
 * sensor, including setting and retrieving the ADC channel configuration,
 * channel number, attenuation, and calibration.
 */
class AnalogSensor : public Sensor {
public:
    /**
     * @brief Constructs an AnalogSensor using an ADC channel.
     * @param channel The ADC channel that the sensor is connected to.
     */
    AnalogSensor(adc_channel_t channel);

    /**
     * @brief Constructs an AnalogSensor using a GPIO pin number.
     * @param pin The GPIO pin that the sensor is connected to.
     */
    AnalogSensor(gpio_num_t pin);

    /**
     * @brief Constructs an AnalogSensor using an ADC channel, with specified
     *        attenuation and resolution.
     * @param channel The ADC channel that the sensor is connected to.
     * @param attenuation The attenuation setting for the ADC.
     * @param resolution The bit resolution of the ADC conversion.
     */
    AnalogSensor(adc_channel_t channel, adc_atten_t attenuation,
                 adc_bitwidth_t resolution);

    /**
     * @brief Constructs an AnalogSensor using a GPIO pin, with specified
     *        attenuation and resolution.
     * @param pin The GPIO pin that the sensor is connected to.
     * @param attenuation The attenuation setting for the ADC.
     * @param resolution The bit resolution of the ADC conversion.
     */
    AnalogSensor(gpio_num_t pin, adc_atten_t attenuation, adc_bitwidth_t resolution);

    /**
     * @brief Destroys the AnalogSensor object.
     */
    virtual ~AnalogSensor();  // Destructor

    /**
     * @brief Initializes the sensor.
     * Load the configuration on the low layer level.
     * @return ESP_OK on success or error code on failure.
     * @note If no further configuration is done, this function will initialize
     * the sensor with default settings: ADC1 unit, 12 bit, no attenuation,
     * default clock. However, adc channel or asociated gpio number must be
     * indicated or the function will return an error.
     */
    esp_err_t init(void) override;

    /**
     * @brief Realizes a single ADC conversion on the specified pin/channel.
     * @return ESP_OK on success, failure code otherwise.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Configures the ADC with default settings:
     * ADC1, Default clock, 11DB atten (3,3 V), 12 bit resolution.
     * @return This function always return ESP_OK, it could change on child's
     * implementation.
     */
    esp_err_t configure(void);

    /**
     * @brief Configures the sensor specifying the ADC channel
     * @param channel Desired ADC channel to configure
     * @param attenuation Desired attenuation (dB) for the input ADC voltage
     * @param resolution Bitwidth resoluton for the ADC conversion output
     * @note init() should be used to apply the configuration.
     */
    esp_err_t configure(adc_channel_t channel, adc_atten_t attenuation,
                        adc_bitwidth_t resolution);

    /**
     * @brief Configures the sensor specifying the GPIO number of the pin
     * @param attenuation Desired attenuation (dB) for the input ADC voltage
     * @param resolution Bitwidth resoluton for the ADC conversion output
     * @param pin Desired GPIO pin to configure
     * @note init() should be used to apply the configuration.
     */
    esp_err_t configure(gpio_num_t pin, adc_atten_t attenuation,
                        adc_bitwidth_t resolution);

    /**
     * @brief Configures the sensor specifying the ADC channel
     * @param unitConfig Structure with the unit configuration
     * @param channel Desired ADC channel to configure
     * @param channelConfig Structure with the channel configuration
     * @note init() should be used to apply the configuration.
     */
    esp_err_t configure(adc_oneshot_unit_init_cfg_t unitConfig, adc_channel_t channel,
                        adc_oneshot_chan_cfg_t channelConfig);

    /**
     * @brief Configures the sensor specifying the GPIO number of the pin
     * @param unitConfig Structure with the unit configuration
     * @param pin Desired GPIO number to configure
     * @param channelConfig Structure with the channel configuration
     * @note init() should be used to apply the configuration.
     */
    esp_err_t configure(adc_oneshot_unit_init_cfg_t unitConfig, gpio_num_t pin,
                        adc_oneshot_chan_cfg_t channelConfig);

    /**
     * @brief Configures the sensor adc unit to be used
     * Check the technical data to determine how many ADCs units are present.
     * @param unitConfig Structure with the configuration of the desired unit.
     * @note init() should be used to apply the configuration.
     */
    esp_err_t configureAdcUnit(adc_oneshot_unit_init_cfg_t unitConfig);

    /**
     * @brief Configures the sensor adc channel to be used
     * @param channelConfig Structure with the values for the attenuation
     * and bidwitdh.
     * @note init() should be used to apply the configuration.
     */
    esp_err_t configureAdcChannel(adc_oneshot_chan_cfg_t channelConfig);

    /**
     * @brief Gets the ADC channel number.
     * @return ADC channel number.
     */
    adc_channel_t getChannelNumber(void) const;

    /**
     * @brief Gets the GPIO number asociated with the sensor/ADC channel
     * @return GPIO number
     */
    gpio_num_t getPinNumber(void) const;

    /**
     * @brief Sets the ADC attenuation.
     * @param attenuation ADC attenuation setting.
     * @note Call init() in order to apply the new changes.
     */
    esp_err_t setAttenuation(adc_atten_t attenuation);

    /**
     * @brief Returns the current ADC attenuation setting.
     */
    adc_atten_t getAttenuation(void) const;

    /**
     * @brief Sets the bit resolution.
     * @param resolution Bit resolution setting for the ADC peripheral.
     * @note Call init() in order to apply the new changes.
     */
    esp_err_t setBitResolution(adc_bitwidth_t resolution);

    /**
     * @brief Returns the current bit resolution setting.
     */
    adc_bitwidth_t getBitResolution(void) const;

    /**
     * @brief Returns the value of the last ADC conversion
     */
    int getValue(void) const;

    /**
     * @brief Returns the previous analog sensor reading
     */
    int getPreviousValue(void) const;

    /**
     * @brief Gets the calibrated voltage value from the last measurement.
     *
     * This function returns the voltage in millivolts (mV) as calculated by the
     * ADC calibration curve. The value is populated by the last call to
     * `measure()` or any of the `read()` methods.
     *
     * @note This function does not perform a new measurement. It only returns
     *       the stored result. If calibration is not supported, is disabled via
     *       `enableCalibration(false)`, or has not been performed yet, this
     *       function will return 0.
     *
     * @return The calibrated voltage in millivolts (mV). Returns 0 if no valid
     *         calibrated reading is available.
     */
    int getCalibratedMv() const;

    /**
     * @brief Performs a measure and returns the obtained value.
     */
    int read(void);

    /**
     * @brief Performs a new measurement and returns the calibrated voltage.
     *
     * This is a convenience function that first calls `measure()` to perform a
     * new ADC conversion and apply calibration, then returns the resulting
     * calibrated voltage in millivolts (mV).
     *
     * @note Calling this function updates the internal state of the sensor,
     *       including the values accessible via `getValue()` and `getCalibratedMv()`.
     *       If calibration is not supported or is disabled, this function will
     *       still perform a raw ADC reading but will return 0.
     *
     * @return The new calibrated voltage in millivolts (mV) from the latest
     *         measurement. Returns 0 if calibration failed or is disabled.
     */
    int readCalibrated();

    /**
     * @brief Returns the max value that the current adc configuration can read.
     */
    int getAdcMaxValue(void) const;

    /**
     * @brief Returns the reference voltage (in mV) used by the ADC unit. It depends on
     * the configured attenuation.
     */
    int getVRef(void) const;

    /**
     * @brief Enables or disables the use of ADC calibration.
     *
     * If calibration is available (supported by the chip), this flag controls
     * whether the calibrated value is used. If disabled, or if calibration is
     * not supported, calculations will fall back to using the raw ADC value.
     *
     * @param enable Set to true to use calibration, false to disable.
     */
    void enableCalibration(bool enable);

    /**
     * @brief Checks if calibration is currently enabled and available.
     * @return true if calibration is active, false otherwise.
     */
    bool isCalibrationEnabled(void) const;

protected:
    /**
     * @brief Configuration structure for initializing the ADC unit.
     *
     * This structure is used to configure the ADC unit with specific settings,
     * including the ADC unit number, clock source, and whether the ADC is
     * controlled by the ULP (Ultra Low Power) coprocessor.
     *
     * @see adc_oneshot_unit_init_cfg_t
     */
    adc_oneshot_unit_init_cfg_t unitConfig_ = {
        ADC_UNIT_1,
        ADC_RTC_CLK_SRC_DEFAULT,
        ADC_ULP_MODE_DISABLE,
    };

    /**
     * @brief Configuration for an ADC channel.
     * Holds the attenuation level and bit width for ADC conversions.
     * This configuration affects the input voltage range and resolution of the
     * ADC readings.
     * @see adc_oneshot_chan_cfg_t
     */
    adc_oneshot_chan_cfg_t channelConfig_ = {ADC_ATTEN_DB_0, ADC_BITWIDTH_DEFAULT};

    adc_cali_curve_fitting_config_t caliConfig_;

    gpio_num_t pin_ = GPIO_NUM_NC;
    adc_channel_t channel_ = (adc_channel_t)ADC_CHANNEL_UNDEFINED;

    adc_oneshot_unit_handle_t adcHandle_ = nullptr;

    adc_cali_handle_t caliHandle_ = nullptr;

    int adcValue_ = kAdcValueError;
    int previousValue_ = kAdcValueError;
    int calibratedVal_ = kAdcValueError;
    int adcMaxValue_ = 3;  // 2^bitwitdh
    int vRef_ = 0;         // mV

    bool useCalibration_ = true;

    /**
     * Sets the corresponding vref based on the attenuation configuration.
     */
    void setVRef(adc_atten_t atten);

private:
    struct CaliKey {
        adc_unit_t unit;
        adc_atten_t atten;
        adc_bitwidth_t bitwidth;

        // Required for std::map to be able to sort the keys
        bool operator<(const CaliKey& other) const {
            return std::tie(unit, atten, bitwidth) <
                   std::tie(other.unit, other.atten, other.bitwidth);
        }
    };

    static std::map<adc_unit_t, adc_oneshot_unit_handle_t> s_unitHandles;

    static std::map<CaliKey, adc_cali_handle_t> s_caliHandles;
};
