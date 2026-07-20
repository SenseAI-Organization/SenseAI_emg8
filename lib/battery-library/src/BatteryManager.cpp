/*****************************************************************************************
 * @file BatteryManager.cpp
 * @brief Implementation of BatteryManager class
 *
 * @version 1.0.0
 * @date 2026-03-18
 * @author <isa@senseai.co>
 ****************************************************************************************/

#include "BatteryManager.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

BatteryManager::BatteryManager(const battery_config_t& config)
    : battery_(nullptr),
      chargingPin_(config.chargingPin),
      pgoodPin_(config.pgoodPin),
      enablePin_(config.enablePin),
      enable5VState_(false),
      isCharging_(false),
      isConnected_(false),
      state_(kDisconnected),
      voltageBuffer_(nullptr),
      percentageBuffer_(nullptr),
      filterSamples_(config.filterSamples),
      filterIndex_(0),
      samplesCollected_(0),
      filteredVoltage_(0),
      filteredPercentage_(0),
      frozenVoltage_(0),
      frozenPercentage_(0),
      prevState_(kDisconnected),
      settlingSamples_(0),
      chargeRampCounter_(0),
      chargeRampInterval_(30),
      initialized_(false) {
    
    // Clamp filter samples to valid range
    if (filterSamples_ < 1) filterSamples_ = 1;
    if (filterSamples_ > 20) filterSamples_ = 20;
    
    // Create Battery instance with specified configuration
    battery_ = new Battery(config.batteryAdcPin, config.r1, config.r2, 
                          config.minVoltage, config.maxVoltage);
    
    if (battery_ != nullptr) {
        battery_->setAttenuation(config.attenuation);
        battery_->setBitResolution(config.resolution);
    }
    
    // Allocate filter buffers
    voltageBuffer_ = new uint16_t[filterSamples_];
    percentageBuffer_ = new uint8_t[filterSamples_];
    
    if (voltageBuffer_ != nullptr) {
        memset(voltageBuffer_, 0, sizeof(uint16_t) * filterSamples_);
    }
    if (percentageBuffer_ != nullptr) {
        memset(percentageBuffer_, 0, sizeof(uint8_t) * filterSamples_);
    }
}

BatteryManager::BatteryManager(gpio_num_t battery_pin, gpio_num_t charging_pin, 
                               gpio_num_t pgood_pin, gpio_num_t enable_pin)
    : battery_(nullptr),
      chargingPin_(charging_pin),
      pgoodPin_(pgood_pin),
      enablePin_(enable_pin),
      enable5VState_(false),
      isCharging_(false),
      isConnected_(false),
      state_(kDisconnected),
      voltageBuffer_(nullptr),
      percentageBuffer_(nullptr),
      filterSamples_(5),  // Default 5 samples
      filterIndex_(0),
      samplesCollected_(0),
      filteredVoltage_(0),
      filteredPercentage_(0),
      frozenVoltage_(0),
      frozenPercentage_(0),
      prevState_(kDisconnected),
      settlingSamples_(0),
      chargeRampCounter_(0),
      chargeRampInterval_(30),
      initialized_(false) {
    
    // Create Battery with default LiPo settings (2.8V-4.062V, 1:1 divider)
    battery_ = new Battery(battery_pin, 10000, 10000, 2800, 4062);
    
    if (battery_ != nullptr) {
        battery_->setAttenuation(ADC_ATTEN_DB_12);
        battery_->setBitResolution(ADC_BITWIDTH_DEFAULT);
    }
    
    // Allocate filter buffers
    voltageBuffer_ = new uint16_t[filterSamples_];
    percentageBuffer_ = new uint8_t[filterSamples_];
    
    if (voltageBuffer_ != nullptr) {
        memset(voltageBuffer_, 0, sizeof(uint16_t) * filterSamples_);
    }
    if (percentageBuffer_ != nullptr) {
        memset(percentageBuffer_, 0, sizeof(uint8_t) * filterSamples_);
    }
}

BatteryManager::~BatteryManager() {
    if (battery_ != nullptr) {
        delete battery_;
        battery_ = nullptr;
    }
    if (voltageBuffer_ != nullptr) {
        delete[] voltageBuffer_;
        voltageBuffer_ = nullptr;
    }
    if (percentageBuffer_ != nullptr) {
        delete[] percentageBuffer_;
        percentageBuffer_ = nullptr;
    }
}

esp_err_t BatteryManager::init(void) {
    if (battery_ == nullptr || voltageBuffer_ == nullptr || percentageBuffer_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize battery voltage sensor
    esp_err_t err = battery_->init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Configure charging pin as input
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << chargingPin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }
    
    // Configure Pgood pin as input
    io_conf.pin_bit_mask = (1ULL << pgoodPin_);
    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    // Configure 5V enable pin as output (driven LOW initially)
    if (enablePin_ != GPIO_NUM_NC) {
        gpio_config_t en_conf = {};
        en_conf.intr_type = GPIO_INTR_DISABLE;
        en_conf.mode = GPIO_MODE_OUTPUT;
        en_conf.pin_bit_mask = (1ULL << enablePin_);
        en_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        en_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        err = gpio_config(&en_conf);
        if (err != ESP_OK) {
            return err;
        }
        gpio_set_level(enablePin_, 0);
        enable5VState_ = false;
    }

    // Establish baseline before filter loop to prevent zero-value issues
    // when device boots already connected to power
    battery_->measure();
    frozenVoltage_ = battery_->getInputVoltage();
    frozenPercentage_ = battery_->getChargePercentage();
    filteredVoltage_ = frozenVoltage_;
    filteredPercentage_ = frozenPercentage_;

    // Populate filter with initial measurements
    for (uint8_t i = 0; i < filterSamples_; i++) {
        measure();
    }
    
    // Update frozen values from populated filter
    frozenVoltage_ = filteredVoltage_;
    frozenPercentage_ = filteredPercentage_;
    
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BatteryManager::measure(void) {
    if (battery_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update connection state from pins
    updateConnectionState();
    
    // Measure battery voltage and percentage
    esp_err_t err = battery_->measure();
    if (err != ESP_OK) {
        return err;
    }
    
    // Get raw readings
    uint16_t raw_voltage = battery_->getInputVoltage();
    uint8_t raw_percentage = battery_->getChargePercentage();
    
    // Update filter with new readings
    updateFilter(raw_voltage, raw_percentage);
    
    return ESP_OK;
}

uint16_t BatteryManager::getVoltage(void) const {
    return filteredVoltage_;
}

uint8_t BatteryManager::getPercentage(void) const {
    return filteredPercentage_;
}

uint16_t BatteryManager::getRawVoltage(void) const {
    if (battery_ == nullptr) {
        return 0;
    }
    return battery_->getInputVoltage();
}

uint8_t BatteryManager::getRawPercentage(void) const {
    if (battery_ == nullptr) {
        return 0;
    }
    return battery_->getChargePercentage();
}

bool BatteryManager::isCharging(void) const {
    return isCharging_;
}

bool BatteryManager::isConnected(void) const {
    return isConnected_;
}

battery_state_t BatteryManager::getState(void) const {
    return state_;
}

const char* BatteryManager::getStateString(void) const {
    switch (state_) {
        case kDisconnected:
            return "Disconnected";
        case kConnected:
            return "Connected";
        case kCharging:
            return "Charging";
        default:
            return "Unknown";
    }
}

esp_err_t BatteryManager::getReport(char* buff) const {
    if (buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    
    snprintf(buff, 256,
             "Battery Report:\n"
             "  State:       %s\n"
             "  Voltage:     %u mV (raw: %u mV)\n"
             "  Charge:      %u%% (raw: %u%%)\n"
             "  Charging:    %s\n"
             "  Connected:   %s\n"
             "  Calibration: %s",
             getStateString(),
             filteredVoltage_,
             getRawVoltage(),
             filteredPercentage_,
             getRawPercentage(),
             isCharging_ ? "YES" : "NO",
             isConnected_ ? "YES" : "NO",
             (battery_ != nullptr && battery_->isCalibrationEnabled()) ? "Enabled" : "Disabled");
    
    return ESP_OK;
}

void BatteryManager::setChargeRampRate(uint16_t samples_per_percent) {
    chargeRampInterval_ = (samples_per_percent < 1) ? 1 : samples_per_percent;
}

void BatteryManager::setFilterSamples(uint8_t samples) {
    if (samples < 1) samples = 1;
    if (samples > 20) samples = 20;
    
    if (samples != filterSamples_) {
        // Reallocate buffers
        if (voltageBuffer_ != nullptr) {
            delete[] voltageBuffer_;
        }
        if (percentageBuffer_ != nullptr) {
            delete[] percentageBuffer_;
        }
        
        filterSamples_ = samples;
        voltageBuffer_ = new uint16_t[filterSamples_];
        percentageBuffer_ = new uint8_t[filterSamples_];
        
        clearFilter();
    }
}

void BatteryManager::clearFilter(void) {
    if (voltageBuffer_ != nullptr) {
        memset(voltageBuffer_, 0, sizeof(uint16_t) * filterSamples_);
    }
    if (percentageBuffer_ != nullptr) {
        memset(percentageBuffer_, 0, sizeof(uint8_t) * filterSamples_);
    }
    filterIndex_ = 0;
    samplesCollected_ = 0;
    filteredVoltage_ = 0;
    filteredPercentage_ = 0;
}

bool BatteryManager::isCriticallyLow(uint16_t threshold_mv) const {
    return filteredVoltage_ < threshold_mv;
}

bool BatteryManager::isFullyCharged(uint8_t threshold_percent) const {
    return (filteredPercentage_ >= threshold_percent) && isConnected_ && !isCharging_;
}

void BatteryManager::updateConnectionState(void) {
    prevState_ = state_;

    int charging_level = gpio_get_level(chargingPin_);
    int pgood_level    = gpio_get_level(pgoodPin_);

    isCharging_  = (charging_level == 0);
    isConnected_ = (pgood_level == 0);

    if (isCharging_) {
        state_ = kCharging;
    } else if (isConnected_) {
        state_ = kConnected;
    } else {
        state_ = kDisconnected;
    }

    if (state_ == prevState_) return;

    // Entering CHARGING: save baseline, start ramp estimation
    if (state_ == kCharging && prevState_ != kCharging) {
        frozenVoltage_    = filteredVoltage_;
        frozenPercentage_ = filteredPercentage_;
        settlingSamples_  = 0;
        chargeRampCounter_ = 0;
    }

    // CHARGING and CONNECTED: charge complete, switch to real ADC readings
    if (prevState_ == kCharging && state_ == kConnected) {
        uint16_t hold_voltage = filteredVoltage_;
        uint8_t hold_percentage = filteredPercentage_;
        clearFilter();
        filteredVoltage_ = hold_voltage;
        filteredPercentage_ = hold_percentage;
    }

    // Any  DISCONNECTED: settling period for surface charge to dissipate
    if (state_ == kDisconnected && prevState_ != kDisconnected) {
        settlingSamples_ = 2;
        uint16_t hold_voltage = filteredVoltage_;
        uint8_t hold_percentage = filteredPercentage_;
        clearFilter();
        filteredVoltage_ = hold_voltage;
        filteredPercentage_ = hold_percentage;
        frozenVoltage_ = hold_voltage;
        frozenPercentage_ = hold_percentage;
    }
}

void BatteryManager::updateFilter(uint16_t voltage, uint8_t percentage) {
    if (voltageBuffer_ == nullptr || percentageBuffer_ == nullptr) return;

    // ── CHARGING: estimate progress by ramping toward 99% ─────────────────
    // ADC reads charger output, not true battery voltage. Estimate by slowly
    // incrementing the displayed percentage. Cap at 99% — only the charger
    // IC knows when it's truly 100% (CONNECTED state).
    if (state_ == kCharging) {
        chargeRampCounter_++;
        if (chargeRampCounter_ >= chargeRampInterval_ && filteredPercentage_ < 99) {
            chargeRampCounter_ = 0;
            filteredPercentage_++;

            // Derive voltage from estimated percentage
            uint16_t min_v = battery_->getMinVoltage();
            uint16_t max_v = battery_->getMaxVoltage();
            filteredVoltage_ = min_v +
                static_cast<uint16_t>(
                    (static_cast<uint32_t>(filteredPercentage_) * (max_v - min_v)) / 100);
        }
        return;
    }

    // ── Settling: hold display values while surface charge dissipates ─────
    if (settlingSamples_ > 0) {
        settlingSamples_--;
        return;
    }

    // ── CONNECTED or DISCONNECTED: normal moving average ──────────────────
    // CONNECTED (charge complete): no charging current, ADC reads real voltage.
    // DISCONNECTED: normal battery reading.
    voltageBuffer_[filterIndex_]    = voltage;
    percentageBuffer_[filterIndex_] = percentage;
    filterIndex_ = (filterIndex_ + 1) % filterSamples_;
    if (samplesCollected_ < filterSamples_) samplesCollected_++;

    filteredVoltage_    = calculateAverage16(voltageBuffer_, samplesCollected_);
    filteredPercentage_ = calculateAverage8(percentageBuffer_,  samplesCollected_);

    frozenVoltage_    = filteredVoltage_;
    frozenPercentage_ = filteredPercentage_;
}

uint16_t BatteryManager::calculateAverage16(const uint16_t* buffer, uint8_t count) const {
    if (buffer == nullptr || count == 0) {
        return 0;
    }
    
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += buffer[i];
    }
    
    return static_cast<uint16_t>(sum / count);
}

uint8_t BatteryManager::calculateAverage8(const uint8_t* buffer, uint8_t count) const {
    if (buffer == nullptr || count == 0) {
        return 0;
    }
    
    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += buffer[i];
    }
    
    return static_cast<uint8_t>(sum / count);
}

void BatteryManager::enable5V(void) {
    if (enablePin_ != GPIO_NUM_NC) {
        gpio_set_level(enablePin_, 1);
        enable5VState_ = true;
    }
}

void BatteryManager::disable5V(void) {
    if (enablePin_ != GPIO_NUM_NC) {
        gpio_set_level(enablePin_, 0);
        enable5VState_ = false;
    }
}

bool BatteryManager::is5VEnabled(void) const {
    return enable5VState_;
}
