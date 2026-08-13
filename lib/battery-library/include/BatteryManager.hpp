/*****************************************************************************************
 * @file BatteryManager.hpp
 * @brief Battery management system with charging and power connection detection
 *
 * This library extends the Battery class to add:
 * - Charging state detection (via charging pin)
 * - Power connection detection (via Pgood pin)
 * - Voltage/percentage stabilization based on connection state
 *
 * @version 1.0.0
 * @date 2026-03-18
 * @author <isa@senseai.co>
 ****************************************************************************************/

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "voltage_divider_sense.hpp"

/**
 * @brief Battery connection states
 */
typedef enum {
    kDisconnected,  ///< Not connected to power, discharging
    kConnected,         ///< Connected to power, not charging
    kCharging           ///< Connected to power and charging
} battery_state_t;

/**
 * @brief Configuration structure for BatteryManager
 */
typedef struct {
    gpio_num_t batteryAdcPin;     ///< ADC pin for battery voltage measurement
    gpio_num_t chargingPin;        ///< GPIO pin for charging detection (active LOW)
    gpio_num_t pgoodPin;           ///< GPIO pin for power good detection (active LOW)
    gpio_num_t enablePin;           ///< GPIO pin for 5V boost enable (GPIO_NUM_NC to disable)
    uint32_t r1;                    ///< Voltage divider resistor R1 (ohms)
    uint32_t r2;                    ///< Voltage divider resistor R2 (ohms)
    uint16_t minVoltage;           ///< Minimum battery voltage (mV) - 0% charge
    uint16_t maxVoltage;           ///< Maximum battery voltage (mV) - 100% charge
    uint8_t filterSamples;         ///< Number of samples for moving average (1-20)
    adc_atten_t attenuation;        ///< ADC attenuation setting
    adc_bitwidth_t resolution;      ///< ADC bit resolution
} battery_config_t;

/**
 * @brief BatteryManager class for comprehensive battery monitoring
 * 
 * Features:
 * - Voltage and percentage reading with calibration support
 * - Charging state detection
 * - Power connection detection
 * - Smart state-aware filtering:
 *   - DISCONNECTED: Normal filtered ADC readings
 *   - CHARGING: Estimates progress, ramping from last known % toward 99%
 *   - CONNECTED (charge complete): Real filtered ADC readings (no charging current)
 * - Settling period after disconnection for accurate surface charge reading
 * - Moving average filter for stable readings
 * 
 * IMPORTANT: During active charging, the ADC reads the charger output voltage
 * (~250mV above battery voltage), NOT the true battery state. The library
 * estimates charging progress by slowly ramping the displayed percentage.
 * When charging completes (CONNECTED state), real readings resume.
 */
class BatteryManager {
   public:
    /**
     * @brief Construct a new BatteryManager object with configuration
     * @param config Battery configuration structure
     */
    explicit BatteryManager(const battery_config_t& config);

    /**
     * @brief Construct with minimal configuration (uses defaults)
     * @param batteryPin ADC pin for battery voltage
     * @param chargingPin GPIO pin for charging detection
     * @param pgoodPin GPIO pin for power good detection
     * @param enablePin GPIO pin for 5V boost enable (GPIO_NUM_NC to skip)
     */
    BatteryManager(gpio_num_t batteryPin, gpio_num_t chargingPin,
                   gpio_num_t pgoodPin, gpio_num_t enablePin = GPIO_NUM_NC);

    /**
     * @brief Destroy the BatteryManager object
     */
    ~BatteryManager();

    /**
     * @brief Initialize battery monitoring system
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t init(void);

    /**
     * @brief Perform a complete battery measurement cycle
     * 
     * Reads voltage, charging state, power state, and updates filtered values
     * 
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t measure(void);

    /**
     * @brief Get current battery voltage in millivolts (filtered and state-aware)
     * 
     * When disconnected: Returns filtered voltage from battery
     * When connected (charge complete): Returns real filtered voltage (ADC is accurate)
     * When charging: Returns estimated voltage ramping toward max
     * 
     * @return Battery voltage in mV
     */
    uint16_t getVoltage(void) const;

    /**
     * @brief Get current battery charge percentage (filtered and state-aware)
     * 
     * When disconnected: Returns filtered percentage based on voltage
     * When connected (charge complete): Returns real filtered percentage
     * When charging: Returns estimated percentage ramping toward 99%
     * 
     * @return Charge percentage (0-100)
     */
    uint8_t getPercentage(void) const;

    /**
     * @brief Get raw (unfiltered) battery voltage
     * @return Raw battery voltage in mV
     */
    uint16_t getRawVoltage(void) const;

    /**
     * @brief Get raw (unfiltered) battery percentage
     * @return Raw charge percentage (0-100)
     */
    uint8_t getRawPercentage(void) const;

    /**
     * @brief Check if battery is currently charging
     * @return true if charging, false otherwise
     */
    bool isCharging(void) const;

    /**
     * @brief Check if device is connected to power
     * @return true if connected, false otherwise
     */
    bool isConnected(void) const;

    /**
     * @brief Get current battery state
     * @return battery_state_t enum value
     */
    battery_state_t getState(void) const;

    /**
     * @brief Get state as human-readable string
     * @return Pointer to state string
     */
    const char* getStateString(void) const;

    /**
     * @brief Get comprehensive battery report
     * @param buff Buffer to write report to (min 256 bytes recommended)
     * @return ESP_OK on success, ESP_ERR_INVALID_ARG if buff is null
     */
    esp_err_t getReport(char* buff) const;

    /**
     * @brief Set the number of samples for moving average filter
     * @param samples Number of samples (1-20, default 5)
     */
    void setFilterSamples(uint8_t samples);

    /**
     * @brief Clear filter history (useful after long sleep or state change)
     */
    void clearFilter(void);

    /**
     * @brief Check if battery voltage is critically low
     * @param threshold_mv Threshold voltage in mV (default 3300)
     * @return true if voltage is below threshold
     */
    bool isCriticallyLow(uint16_t threshold_mv = 3300) const;

    /**
     * @brief Check if battery is fully charged
     * @param threshold_percent Threshold percentage (default 95)
     * @return true if percentage is above threshold AND connected
     */
    bool isFullyCharged(uint8_t threshold_percent = 95) const;

    /**
     * @brief Enable the 5V boost output.
     *
     * Drives the enable pin HIGH. No-op if enablePin was set to GPIO_NUM_NC.
     */
    void enable5V(void);

    /**
     * @brief Disable the 5V boost output.
     *
     * Drives the enable pin LOW. No-op if enablePin was set to GPIO_NUM_NC.
     */
    void disable5V(void);

    /**
     * @brief Check whether the 5V boost output is currently enabled.
     * @return true if enabled, false otherwise.
     */
    bool is5VEnabled(void) const;

    /**
     * @brief Set how many measure() calls per 1% increment while charging
     * 
     * Default is 30 samples. At 2s intervals: 1% every 60s, ~30min from 70%->100%.
     * Tune based on your measurement interval and battery capacity.
     * 
     * @param samples_per_percent Number of measure() calls per 1% bump (min 1)
     */
    void setChargeRampRate(uint16_t samples_per_percent);

   private:
    Battery* battery_;                    ///< Battery voltage/percentage sensor
    gpio_num_t chargingPin_;             ///< GPIO for charging detection
    gpio_num_t pgoodPin_;                ///< GPIO for power good detection
    gpio_num_t enablePin_;               ///< GPIO for 5V boost enable
    bool enable5VState_;                 ///< Current state of 5V output
    
    // Current state
    bool isCharging_;                    ///< Current charging state
    bool isConnected_;                   ///< Current connection state
    battery_state_t state_;               ///< Current battery state
    
    // Filter storage
    uint16_t* voltageBuffer_;            ///< Circular buffer for voltage samples
    uint8_t* percentageBuffer_;          ///< Circular buffer for percentage samples
    uint8_t filterSamples_;              ///< Number of samples in filter
    uint8_t filterIndex_;                ///< Current index in circular buffer
    uint8_t samplesCollected_;           ///< Samples collected so far
    
    // Filtered values
    uint16_t filteredVoltage_;           ///< Moving average voltage
    uint8_t filteredPercentage_;         ///< Moving average percentage
    
    // State-aware tracking for accurate readings
    uint16_t frozenVoltage_;             ///< Last known good voltage when disconnected
    uint8_t frozenPercentage_;           ///< Last known good percentage when disconnected
    battery_state_t prevState_;          ///< Previous battery state for transition detection
    uint8_t settlingSamples_;            ///< Samples to skip after disconnecting (surface charge settling)

    // Charging ramp estimation
    uint16_t chargeRampCounter_;         ///< Counts measure() calls during charging
    uint16_t chargeRampInterval_;        ///< Samples per 1% increment during charging

    bool initialized_;                    ///< Initialization flag

    /**
     * @brief Read and update charging and power connection states
     */
    void updateConnectionState(void);

    /**
     * @brief Add new sample to filter and update averaged values
     * @param voltage Raw voltage reading
     * @param percentage Raw percentage reading
     */
    void updateFilter(uint16_t voltage, uint8_t percentage);

    /**
     * @brief Calculate moving average from buffer
     * @param buffer Pointer to uint16_t buffer
     * @param count Number of valid samples
     * @return Average value
     */
    uint16_t calculateAverage16(const uint16_t* buffer, uint8_t count) const;

    /**
     * @brief Calculate moving average from buffer
     * @param buffer Pointer to uint8_t buffer
     * @param count Number of valid samples
     * @return Average value
     */
    uint8_t calculateAverage8(const uint8_t* buffer, uint8_t count) const;
};

// Default battery configuration constants
namespace {

// Pin configuration
constexpr gpio_num_t kDefaultBatteryAdcPin = GPIO_NUM_8;
constexpr gpio_num_t kDefaultChargingPin   = GPIO_NUM_9;
constexpr gpio_num_t kDefaultPgoodPin      = GPIO_NUM_10;

// Voltage divider (1:1 ratio, 10kΩ:10kΩ)
constexpr uint32_t kDefaultR1 = 10000;
constexpr uint32_t kDefaultR2 = 10000;

// LiPo voltage range (mV)
constexpr uint16_t kDefaultMinVoltage = 2800;
constexpr uint16_t kDefaultMaxVoltage = 4062;

// Filter configuration
constexpr uint8_t kDefaultFilterSamples = 5;

// ADC configuration
constexpr adc_atten_t kDefaultAttenuation     = ADC_ATTEN_DB_12;  // 0-3.3V range
constexpr adc_bitwidth_t kDefaultResolution   = ADC_BITWIDTH_DEFAULT;  // 12-bit

}  // namespace
