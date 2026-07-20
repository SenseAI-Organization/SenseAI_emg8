/*******************************************************************************
 * @file ADS1015.hpp
 * @brief Contains the declarations of the ADS1015 class methods.
 *
 * This device is a four channel ADC with 12-bit resolution and a programmable
 * gain amplifier. The device can be configured to use one of four I2C addresses.
 *
 * @version v0.1.0
 * @date 2026-04-11
 * @author daniel@sense-ai.co, Sense AI
 *******************************************************************************/

#pragma once

#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "smart_sensor_sense.hpp"

/**
 * @class ADS1015
 * @brief Class for the ADS1015 ADC.
 */

class ADS1015 : public Sensor {
public:
    static constexpr uint8_t kMaxChannels = 4;
    static constexpr uint8_t kMaxActiveChannels = 4;

    /**
     * @brief Conversion callback. timestampUs is captured in the DRDY ISR
     * (µs, lower 32 bits of esp_timer_get_time()), so it reflects conversion
     * completion time rather than task service time.
     */
    typedef void (*ConversionCallback)(uint8_t channel, int16_t value,
                                       uint32_t timestampUs, void* arg);

    /**
     * @enum ADS111X_Address
     * @brief Enumeration for the device I2C addresses.
     */
    enum ADS111X_Address {
        ADS111X_ADDR_GND =
            0x48,  //!< I2C device address with ADDR pin connected to ground
        ADS111X_ADDR_VCC = 0x49,  //!< I2C device address with ADDR pin connected to VCC
        ADS111X_ADDR_SDA = 0x4a,  //!< I2C device address with ADDR pin connected to SDA
        ADS111X_ADDR_SCL = 0x4b   //!< I2C device address with ADDR pin connected to SCL
    };

    /**
     * @enum ConfigMode
     * @brief Enumeration for the device operating mode.
     */
    enum class ConfigOS : uint16_t {
        No = 0x0000,
        Single = 0x8000,
        NotReady = 0x0000,
        Ready = 0x8000
    };

    /**
     * @enum ConfigMode
     * @brief Enumeration for the device operating mode.
     */
    enum class ConfigMode : uint16_t { Continuous = 0x0000, Single = 0x0100 };

    /**
     * @enum ConfigMux
     * @brief Enumeration for the input multiplexer configuration.
     */
    enum class ConfigMux : uint16_t {
        Single_0 = 0x4000,
        Single_1 = 0x5000,
        Single_2 = 0x6000,
        Single_3 = 0x7000,
        Diff_P0_N1 = 0x0000,
        Diff_P0_N3 = 0x1000,
        Diff_P1_N3 = 0x2000,
        Diff_P2_N3 = 0x3000
    };

    /**
     * @enum ConfigRate
     * @brief Enumeration for the data rate.
     */
    enum class ConfigRate : uint16_t {
        Rate_128Hz = 0x0000,
        Rate_250Hz = 0x0020,
        Rate_490Hz = 0x0040,
        Rate_920Hz = 0x0060,
        Rate_1600Hz = 0x0080,
        Rate_2400Hz = 0x00A0,
        Rate_3300Hz = 0x00C0
    };

    /**
     * @enum ConfigPGA
     * @brief Enumeration for the programmable gain amplifier configuration.
     */
    enum class ConfigPGA : uint16_t {
        TwoThirds = 0x0000,
        One = 0x0200,
        Two = 0x0400,
        Four = 0x0600,
        Eight = 0x0800,
        Sixteen = 0x0A00
    };

    /**
     * @enum ConfigComparatorMode
     * @brief Enumeration for the comparator mode.
     */
    enum class ConfigComparatorMode : uint16_t { Traditional = 0x0000, Window = 0x0010 };

    /**
     * @enum ConfigComparatorPolarity
     * @brief Enumeration for the comparator polarity.
     */
    enum class ConfigComparatorPolarity : uint16_t {
        ActiveLow = 0x0000,
        ActiveHigh = 0x0008
    };

    /**
     * @enum ConfigComparatorLatching
     * @brief Enumeration for the comparator latching.
     */
    enum class ConfigComparatorLatching : uint16_t {
        NonLatching = 0x0000,
        Latching = 0x0004
    };

    /**
     * @enum ConfigComparatorQueue
     * @brief Enumeration for the comparator queue.
     */
    enum class ConfigComparatorQueue : uint16_t {
        OneConversion = 0x0000,
        TwoConversions = 0x0001,
        FourConversions = 0x0002,
        None = 0x0003
    };

    /**
     * @enum Register
     * @brief Enumeration for the register addresses.
     */
    enum class Register : uint8_t {
        Conversion = 0,
        Config = 1,
        ThreshLow = 2,
        ThreshHigh = 3
    };

    /**
     * @enum ConfigMask
     * @brief Enumeration for the configuration masks.
     */
    enum class ConfigMask : uint16_t {
        CompQue = 0x03,
        CompLat = 0x01,
        CompPol = 0x01,
        CompMode = 0x01,
        DataRate = 0x07,
        Mode = 0x01,
        PGA = 0x07,
        Mux = 0x07,
        OS = 0x01
    };

    /**
     * @enum ConfigOffset
     * @brief Enumeration for the configuration offsets.
     */
    enum class ConfigOffset : uint8_t {
        CompQue = 0,
        CompLat = 2,
        CompPol = 3,
        CompMode = 4,
        DataRate = 5,
        Mode = 8,
        PGA = 9,
        Mux = 12,
        OS = 15
    };

    /**
     * @brief Constructor for ADS1015.
     * @param i2cInstance Reference to the I2C instance.
     * @param address I2C address of the device.
     */
    ADS1015(I2C& i2cInstance, uint8_t address = ADS111X_Address::ADS111X_ADDR_GND);

    /**
     * @brief Destructor for ADS1015.
     */
    ~ADS1015();

    /**
     * @brief Initialize the ADS1015 device.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t init(void) override;

    /**
     * @brief Initialize the ADS1015 device with specific gain and sample frequency.
     * @param gain Gain setting for the ADC.
     * @param sampleFrequency Sample frequency setting for the ADC.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t init(uint8_t gain, uint8_t sampleFrequency);

    /**
     * @brief Perform a measurement on all channels.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Perform a measurement on a specific channel.
     * @param channel Channel to measure.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t measure(uint8_t channel);

    /**
     * @brief Get the ID of the ADS1015 device.
     * @return uint8_t Device ID.
     */
    uint8_t getID(void) const override;

    /**
     * @brief Get a report of the ADS1015 device status.
     * @param _buff Buffer to store the report.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t getReport(char* _buff) const override;

    /**
     * @brief Get the measurement value of a specific channel.
     * @param channel Channel to get the measurement from.
     * @return int16_t Measurement value.
     */
    int16_t getChannel(uint8_t channel);

    /**
     * @brief Set the configuration of the ADS1015 device.
     * @param channel Channel to configure.
     * @param gain Gain setting for the ADC.
     * @param sampleFrequency Sample frequency setting for the ADC.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t setConfig(uint8_t channel, uint8_t gain, uint8_t sampleFrequency);

    /**
     * @brief Check if the ADS1015 device is available.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t checkForDevice();

    /**
     * @brief Read a single-ended value from a specific channel.
     * @param channel Channel to read from.
     * @return int8_t Measurement value.
     */
    int8_t readSingleEnded(uint8_t channel);

    /**
     * @brief Read a single-ended signed value from a specific channel.
     * @param channel Channel to read from.
     * @return int16_t Measurement value.
     */
    int16_t readSingleEndedSigned(uint8_t channel);

    // ─── Continuous / ALERT-RDY Mode ─────────────────────────────────────────────

    /**
     * @brief Configure the ALERT/RDY pin as a data-ready interrupt source.
     *
     * Sets ThreshLo=0x0000 and ThreshHi=0x8000, which turns the ALERT pin
     * into a conversion-ready signal that pulses after every conversion.
     *
     * @param alertPin GPIO connected to ALERT/RDY.
     * @param activeLow true = ALERT is active-low (default ADS1015 behavior).
     * @return esp_err_t
     */
    esp_err_t configureAlertPin(gpio_num_t alertPin, bool activeLow = true);

    /**
     * @brief Start continuous round-robin sampling on the given channels.
     *
     * The driver will cycle through the channels at the configured data rate,
     * using the ALERT/RDY interrupt to trigger each read + MUX switch.
     * Conversions happen at the hardware data-rate (up to 3300 SPS).
     * For 2 channels at 3300 SPS → ~1650 SPS per channel.
     *
     * @param channels  Array of channel indices (0-3).
     * @param numChannels Number of channels in the array (1-4).
     * @param rate Data rate for conversions.
     * @param gain PGA gain for conversions.
     * @return esp_err_t
     */
    esp_err_t startContinuous(const uint8_t* channels, uint8_t numChannels,
                              ConfigRate rate = ConfigRate::Rate_3300Hz,
                              ConfigPGA gain = ConfigPGA::One);

    /**
     * @brief Stop continuous mode and return to idle (single-shot power-down).
     * @return esp_err_t
     */
    esp_err_t stopContinuous();

    /**
     * @brief Check if continuous mode is currently running.
     * @return true if running.
     */
    bool isContinuousRunning() const;

    /**
     * @brief Get the latest conversion result for a channel (non-blocking).
     * @param channel Channel index (0-3).
     * @return int16_t Latest signed 12-bit value, or 0 if never read.
     */
    int16_t getLatestReading(uint8_t channel) const;

    /**
     * @brief Register a callback invoked every time a new conversion completes.
     *
     * Called from the internal handler task context (not ISR).
     *
     * @param cb Callback function (channel, value, arg).
     * @param arg User argument passed to callback.
     */
    void onConversion(ConversionCallback cb, void* arg = nullptr);

    // ─── Mixed-Rate Continuous Mode ──────────────────────────────────────────────

    /**
     * @brief Channel descriptor for mixed-rate continuous sampling.
     *
     * Allows different channels to be sampled at different effective rates.
     * The `divider` field controls how often that channel is inserted into the
     * round-robin: divider=1 means every cycle, divider=N means once every
     * N fast-channel cycles.
     */
    struct ChannelConfig {
        uint8_t channel;  ///< ADC channel index (0-3)
        uint8_t divider;  ///< Sample every N fast-cycles (1 = fast, N>1 = slow)
        ConfigPGA gain;   ///< Per-channel PGA gain
    };

    /**
     * @brief Start mixed-rate continuous sampling.
     *
     * Optimized for scenarios like EMG acquisition where some channels need
     * maximum sample rate (e.g. raw EMG) while others only need low rates
     * (e.g. filtered envelope).
     *
     * The scheduler interleaves fast channels (divider=1) in tight round-robin
     * and injects slow channels (divider>1) periodically. For example, with
     * 2 fast EMG channels and 2 slow envelope channels (divider=26), at
     * 3300 SPS you get ~1300 Hz per EMG channel and ~50 Hz per envelope.
     *
     * @param configs  Array of ChannelConfig descriptors.
     * @param numConfigs Number of entries (1-4).
     * @param rate     Hardware data rate (shared — use the highest needed).
     * @return esp_err_t
     */
    esp_err_t startMixedContinuous(const ChannelConfig* configs, uint8_t numConfigs,
                                   ConfigRate rate = ConfigRate::Rate_3300Hz);

    /**
     * @brief Start mixed-rate continuous sampling without creating an internal task.
     *
     * Sets up ISR, channel scheduling, and kicks off the first conversion,
     * but does NOT create a FreeRTOS task. The caller must poll
     * serviceConversion() from their own task.
     *
     * @param configs  Array of ChannelConfig descriptors.
     * @param numConfigs Number of entries (1-4).
     * @param rate     Hardware data rate (shared — use the highest needed).
     * @return esp_err_t
     */
    esp_err_t startMixedContinuousExternal(const ChannelConfig* configs,
                                           uint8_t numConfigs,
                                           ConfigRate rate = ConfigRate::Rate_3300Hz);

    /**
     * @brief Service one pending conversion (non-blocking).
     *
     * Checks the DRDY semaphore. If a conversion is ready, reads it,
     * invokes the callback, and switches the MUX to the next channel.
     * Designed to be called in a loop from an external task that handles
     * multiple ADS1015 instances.
     *
     * @return true if a conversion was serviced, false if none pending.
     */
    bool serviceConversion();

    /**
     * @brief Get the actual effective sample rate for a channel in mixed mode.
     *
     * This is an estimate based on the data rate, I2C speed, and scheduling.
     * Only valid after startMixedContinuous() is running.
     *
     * @param channel Channel index.
     * @return float Estimated Hz, or 0 if channel is not active.
     */
    float getEffectiveSampleRate(uint8_t channel) const;

private:
    I2C& i2c_;                 ///< Reference to the I2C instance.
    ADS111X_Address address_;  ///< I2C address of the device.

    /**
     * @brief Check if a sample is ready.
     * @param sampleReady Pointer to a boolean to store the result.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t available(bool* sampleReady);

    /**
     * @brief Convert an unsigned value to a signed value.
     * @param value Unsigned value to convert.
     * @return int16_t Signed value.
     */
    int16_t convertUnsignedToSigned(uint16_t value);

    /**
     * @brief Read configuration bits from the ADS1015 device.
     * @param data Pointer to store the configuration bits.
     * @return esp_err_t Error code indicating success or failure.
     */
    esp_err_t readConfBits(uint8_t* data);

    /**
     * @brief Set the sample frequency in the configuration.
     * @param config Current configuration.
     * @param sampleFrequency Sample frequency to set.
     * @return uint16_t Updated configuration.
     */
    uint16_t setSampleFrequency(uint16_t config, uint8_t sampleFrequency);

    /**
     * @brief Set the gain in the configuration.
     * @param config Current configuration.
     * @param gain Gain to set.
     * @return uint16_t Updated configuration.
     */
    uint16_t setGain(uint16_t config, uint8_t gain);

    /**
     * @brief Set the channel in the configuration.
     * @param config Current configuration.
     * @param channel Channel to set.
     * @return uint16_t Updated configuration.
     */
    uint16_t setChannel(uint16_t config, uint8_t channel);

    uint8_t gain_ = 1;             ///< Gain setting for the ADC.
    uint8_t sampleFrequency_ = 6;  ///< Sample frequency setting for the ADC.

    struct adcRawData {
        uint8_t channels[8];  ///< Raw data for the ADC channels.
    };
    adcRawData rawData = {{0, 0, 0, 0, 0, 0, 0, 0}};

    struct adcData {
        uint16_t channels[4];  ///< Processed data for the ADC channels.
    };
    adcData adcData = {0, 0, 0, 0};

    // ─── Continuous mode state ───────────────────────────────────────────────

    /**
     * @brief Build a full 16-bit config word for continuous mode on a given mux.
     */
    uint16_t buildContinuousConfig(ConfigMux mux, ConfigRate rate, ConfigPGA gain);

    /**
     * @brief Write config register with given 16-bit value.
     */
    esp_err_t writeConfig(uint16_t config);

    /**
     * @brief Write a 16-bit value to a register.
     */
    esp_err_t writeRegister(Register reg, uint16_t value);

    /**
     * @brief Read the 16-bit conversion register and return the signed 12-bit result.
     */
    int16_t readConversionResult();

    /**
     * @brief Map channel index (0-3) to ConfigMux for single-ended input.
     */
    static ConfigMux channelToMux(uint8_t channel);

    /**
     * @brief Internal FreeRTOS task that handles DRDY interrupts.
     */
    static void continuousTask(void* pvParam);

    /**
     * @brief ISR handler for the ALERT/RDY pin.
     */
    static void IRAM_ATTR alertISR(void* arg);

    gpio_num_t alertPin_ = GPIO_NUM_NC;                  ///< ALERT/RDY GPIO pin
    volatile bool continuousRunning_ = false;            ///< Flag: continuous mode active
    volatile int16_t latestReading_[kMaxChannels] = {};  ///< Latest per-channel values
    volatile uint32_t drdyTimestampUs_ = 0;  ///< µs timestamp of last DRDY edge (set in ISR)

    uint8_t activeChannels_[kMaxActiveChannels] = {};  ///< Channels to cycle through
    uint8_t numActiveChannels_ = 0;                    ///< Number of active channels
    uint8_t currentMuxIndex_ = 0;                      ///< Index into activeChannels_

    ConfigRate continuousRate_ = ConfigRate::Rate_3300Hz;
    ConfigPGA continuousPGA_ = ConfigPGA::One;

    SemaphoreHandle_t drdySemaphore_ = nullptr;  ///< Given by ISR on DRDY
    TaskHandle_t continuousTaskHandle_ = nullptr;

    ConversionCallback convCallback_ = nullptr;
    void* convCallbackArg_ = nullptr;

    // ─── Mixed-rate scheduling state ─────────────────────────────────────────

    /**
     * @brief Internal FreeRTOS task for mixed-rate continuous mode.
     */
    static void mixedContinuousTask(void* pvParam);

    /**
     * @brief Decide which channel to convert next in mixed-rate mode.
     * @return channel index (0-3)
     */
    uint8_t nextMixedChannel();

    ChannelConfig channelConfigs_[kMaxActiveChannels] = {};
    uint8_t numChannelConfigs_ = 0;

    uint8_t fastChannels_[kMaxActiveChannels] = {};
    uint8_t numFastChannels_ = 0;

    uint8_t slowChannels_[kMaxActiveChannels] = {};
    uint8_t slowDividers_[kMaxActiveChannels] = {};
    uint32_t slowNextDue_[kMaxActiveChannels] = {};  ///< Fast-cycle count at which each slow channel fires next
    uint8_t numSlowChannels_ = 0;

    uint32_t fastCycleCount_ = 0;  ///< Counts fast round-robin cycles
    uint8_t fastIndex_ = 0;        ///< Index into fastChannels_
    uint8_t slowIndex_ = 0;        ///< Next slow channel to inject

    bool mixedMode_ = false;  ///< True if using mixed-rate mode
};
