/*******************************************************************************
 * @file smart_sensor_sense.hpp
 * @brief Contains the declarations to work with the I2C peripheral.
 *
 * @version v0.1.0
 * @date 2024-05-14
 * @author Sense-AI
 *******************************************************************************/

#pragma once

#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "sensors_sense.hpp"

/******************************************************************************/
/*                                 I2C                                        */
/******************************************************************************/

/**
 * @class I2C
 * @brief A class to interface with the I2C peripheral.
 */
class I2C {
public:
    /**
     * @brief Constructor for the I2C class.
     * @param port I2C port number.
     * @param sdaPin GPIO number for SDA pin.
     * @param sclPin GPIO number for SCL pin.
     * @param frequency I2C bus frequency.
     * @param internalPullup Flag to use internal pull-up resistors.
     */
    I2C(i2c_port_t port, gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t frequency,
        bool internalPullup);

    /**
     * @brief Destructor for the I2C class.
     */
    ~I2C();

    /**
     * @brief Initializes the I2C peripheral.
     * @return esp_err_t Error status.
     */
    esp_err_t init();

    /**
     * @brief Deinitializes the I2C peripheral.
     */
    void deinit();

    /**
     * @brief Writes data to an I2C device.
     * @param deviceAddress I2C address of the device.
     * @param registerAddress Register address to write to.
     * @param data Pointer to data to write.
     * @param len Length of data to write.
     * @return esp_err_t Error status.
     */
    esp_err_t write(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data,
                    size_t len);

    /**
     * @brief Reads data from an I2C device.
     * @param deviceAddress I2C address of the device.
     * @param registerAddress Register address to read from.
     * @param data Pointer to buffer to store read data.
     * @param len Length of data to read.
     * @return esp_err_t Error status.
     */
    esp_err_t read(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data,
                   size_t len);

    /**
     * @brief Checks whether a device ACKs its address on the bus.
     * @param deviceAddress I2C address of the device.
     * @return esp_err_t ESP_OK if the device responded.
     */
    esp_err_t probe(uint8_t deviceAddress);

    /**
     * @brief Clears the I2C bus.
     */
    void clearBus(void);

private:
    static constexpr uint8_t kMaxDevices = 8;   /**< Device-handle cache size */
    static constexpr int kTimeoutMs = 20;       /**< Per-transaction timeout */

    /**
     * @brief Returns the cached device handle for an address, registering it
     * on first use. Registration is not thread-safe; first contact with each
     * device must happen before concurrent bus users start (transactions
     * themselves are serialized by the driver).
     */
    i2c_master_dev_handle_t deviceFor(uint8_t deviceAddress);

    i2c_port_t port_;       /**< I2C port number */
    gpio_num_t sdaPin_;     /**< GPIO number for SDA pin */
    gpio_num_t sclPin_;     /**< GPIO number for SCL pin */
    uint32_t frequency_;    /**< I2C bus frequency */
    bool internalResistor_; /**< Flag for internal pull-up resistors */

    i2c_master_bus_handle_t busHandle_ = nullptr;
    uint8_t devAddrs_[kMaxDevices] = {};
    i2c_master_dev_handle_t devHandles_[kMaxDevices] = {};
    uint8_t numDevices_ = 0;
};

/******************************************************************************/
/*                                 Timer                                      */
/******************************************************************************/

/**
 * @class Timer
 * @brief A class to encapsulate Timer functionality.
 *
 * This class uses a fixed resolution of 1 MHz (1 tick per microsecond).
 * The default constructor parameter specifies the period between interrupts in
 * milliseconds. Additional methods allow reconfiguration using a desired
 * frequency (Hz) or period (ms), with corresponding getters.
 */
class Timer {
public:
    /**
     * @brief Callback type for timer alarms.
     * The callback function should accept a void pointer argument.
     */
    typedef void (*timer_callback_t)(void* arg);

    /**
     * @brief Constructor for the Timer class.
     * @param periodMs Period between interrupts in milliseconds.
     * @param intrPriority Interrupt priority for the timer.
     * @param autoReload Set to true if the timer should automatically reload
     *                   on alarm.
     */
    Timer(uint32_t periodMs, int intrPriority, bool autoReload);

    /**
     * @brief Constructor for Timer class with only the period parameter.
     *
     * This constructor uses a default interrupt priority of 1 and auto-reload enabled.
     *
     * @param periodMs Period between interrupts in milliseconds.
     */
    Timer(uint32_t periodMs);

    /**
     * @brief Default constructor for Timer with default values.
     *
     * Constructs a Timer with a default period of 250 ms, an interrupt priority of 1,
     * and auto-reload enabled.
     */
    Timer();

    /**
     * @brief Destructor for the Timer class.
     */
    ~Timer();

    /**
     * @brief Initializes the timer peripheral.
     * @return esp_err_t Error status.
     */
    esp_err_t init(void);

    /**
     * @brief Configures the timer alarm based on the internal alarm_count.
     * @return esp_err_t Error status.
     */
    esp_err_t configure(void);

    /**
     * @brief Sets the callback function to be executed on timer alarm.
     * @param callback Callback function pointer.
     * @param arg Argument to pass to the callback function.
     */
    void setCallback(timer_callback_t callback, void* arg = nullptr);

    /**
     * @brief Starts the timer counter.
     * @return esp_err_t Error status.
     */
    esp_err_t start(void);

    /**
     * @brief Stops the timer counter.
     * @return esp_err_t Error status.
     */
    esp_err_t stop(void);

    /**
     * @brief Deinitializes the timer peripheral.
     * @return esp_err_t Error status.
     */
    esp_err_t deinit(void);

    /**
     * @brief Gets the current counter value (ticks) on cnt register.
     * @param count Pointer to store the counter value.
     * @return esp_err_t Error status.
     */
    esp_err_t getCounter(uint64_t* count) const;

    /**
     * @brief Sets the current counter value.
     * @param count New counter value.
     * @return esp_err_t Error status.
     */
    esp_err_t setCounter(uint64_t count) const;

    /**
     * @brief Gets the current counter value in microseconds.
     * @param us Pointer to store the counter value in us.
     * @return esp_err_t Error status.
     */
    esp_err_t getCounterUs(uint32_t* us) const;

    /**
     * @brief Gets the current counter value in milliseconds.
     * @param ms Pointer to store the counter value in ms.
     * @return esp_err_t Error status.
     */
    esp_err_t getCounterMs(uint32_t* ms) const;

    /**
     * @brief Resets the counter value to 0.
     * @return esp_err_t Error status.
     */
    esp_err_t reset(void) const;

    /**
     * @brief Sets the timer alarm period based on a desired frequency in Hz.
     * @param frequency Frequency in Hz.
     * @return esp_err_t Error status.
     */
    esp_err_t setFrequency(uint32_t frequency);

    /**
     * @brief Returns the current frequency in Hz.
     * @return uint32_t Frequency in Hz.
     */
    uint32_t getFrequency(void) const;

    /**
     * @brief Sets the timer alarm period in milliseconds.
     * @param periodMs Period in milliseconds.
     * @return esp_err_t Error status.
     */
    esp_err_t setPeriodMs(uint32_t periodMs);

    /**
     * @brief Returns the current period in milliseconds.
     * @return uint32_t Period in milliseconds.
     */
    uint32_t getPeriodMs(void) const;

    /**
     * @brief Enables the timer interrupt callback.
     */
    void enableInterrupts(void);

    /**
     * @brief Disables the timer interrupt callback.
     */
    void disableInterrupts(void);

    /**
     * @brief Enables sleep mode for the timer.
     *
     * When enabled, the timer's power domain can be turned off during deep sleep.
     * This function stops the timer, deinitializes it, and reinitializes with the
     * appropriate configuration.
     *
     * @return esp_err_t Error status.
     * @note The counter is disabled after this function is called, enable manually
     * in order to make it count again. Also, the interrupts are disabled and
     * re-enabled in the process. Disable manually if needed after calling this
     * function.
     */
    esp_err_t enableSleepMode(void);

    /**
     * @brief Disables sleep mode for the timer.
     *
     * When disabled, the timer remains powered even during deep sleep.
     * This function stops the timer, deinitializes it, and reinitializes with the
     * appropriate configuration.
     *
     * @return esp_err_t Error status.
     */
    esp_err_t disableSleepMode(void);

private:
    static const uint64_t kResolutionHz = 1000000;  // Fixed resolution: 1 MHz

    gptimer_handle_t timerHandle_;
    uint64_t alarmCount_;  // Calculated alarm count in ticks
    int intrPriority_;
    bool autoReload_;
    timer_callback_t userCallback_;
    void* callbackArg_;
    bool interruptsEnabled_;  // Flag to enable/disable callback execution
    bool sleepModeEnabled_;   // Flag to en/disable the power source on sleep

    /**
     * @brief Static ISR callback function for GPTimer.
     * @param timer GPTimer handle.
     * @param eventData Alarm event data.
     * @param userData Pointer to the Timer instance.
     * @return true to acknowledge the alarm.
     */
    static bool IRAM_ATTR timerISR(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t* eventData,
                                   void* userData);
};
