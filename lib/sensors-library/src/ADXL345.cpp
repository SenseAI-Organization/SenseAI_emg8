/*******************************************************************************
 *******************************************************************************
 * @file ADXL345.cpp
 * @brief Contains the definitions of the ADXL345 class methods.
 *
 * This sensor has 3 axis accelerometers and can generate up to 2 interrupts by
 * different pins.
 *
 * @note Changed the init function v0.1.1, now it doesn't calibrate the device.
 *
 * @version v0.2.0
 * @date 2025-03-13
 * @author emmanuel@sense-ai.co, Sense AI
 * @author mateor@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "ADXL345.hpp"
#include "esp_system.h"

bool ADXL345::isIsrServiceInstalled_ = false;

ADXL345::ActivityControl operator|(ADXL345::ActivityControl a,
                                   ADXL345::ActivityControl b) {
    return static_cast<ADXL345::ActivityControl>(static_cast<uint8_t>(a) |
                                                 static_cast<uint8_t>(b));
}

ADXL345::ActivityControl operator&(ADXL345::ActivityControl a,
                                   ADXL345::ActivityControl b) {
    return static_cast<ADXL345::ActivityControl>(static_cast<uint8_t>(a) &
                                                 static_cast<uint8_t>(b));
}

ADXL345::ActivityControl operator~(ADXL345::ActivityControl a) {
    return static_cast<ADXL345::ActivityControl>(~static_cast<uint8_t>(a));
}

ADXL345::PowerControlBits operator|(ADXL345::PowerControlBits a,
                                    ADXL345::PowerControlBits b) {
    return static_cast<ADXL345::PowerControlBits>(static_cast<uint8_t>(a) |
                                                  static_cast<uint8_t>(b));
}

ADXL345::PowerControlBits operator&(ADXL345::PowerControlBits a,
                                    ADXL345::PowerControlBits b) {
    return static_cast<ADXL345::PowerControlBits>(static_cast<uint8_t>(a) &
                                                  static_cast<uint8_t>(b));
}

ADXL345::PowerControlBits operator~(ADXL345::PowerControlBits a) {
    return static_cast<ADXL345::PowerControlBits>(~static_cast<uint8_t>(a));
}

ADXL345::InterruptFlags operator|(ADXL345::InterruptFlags a, ADXL345::InterruptFlags b) {
    return static_cast<ADXL345::InterruptFlags>(static_cast<uint8_t>(a) |
                                                static_cast<uint8_t>(b));
}

ADXL345::InterruptFlags operator&(ADXL345::InterruptFlags a, ADXL345::InterruptFlags b) {
    return static_cast<ADXL345::InterruptFlags>(static_cast<uint8_t>(a) &
                                                static_cast<uint8_t>(b));
}

ADXL345::ADXL345(I2C& i2cInstance, uint8_t slaveAddress)
    : i2c_(i2cInstance), slaveAddress_(slaveAddress) {
}

ADXL345::~ADXL345() {
}

esp_err_t ADXL345::init(void) {
    uint8_t id = readDeviceID();
    if (id != kDeviceId) {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t ADXL345::measure(void) {
    uint8_t rawData[6] = {0};

    esp_err_t err = i2c_.read(slaveAddress_, kDataX0Reg, rawData, 6);
    if (err) {
        return err;
    }

    for (uint8_t i = 0; i < 3; i++) {
        accelRaw_[i] = ((int16_t)rawData[(2 * i) + 1] << 8) | rawData[2 * i];
    }

    return ESP_OK;
}

esp_err_t ADXL345::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 128, "ID: %X, ax: %d, ay: %d, az: %d", getID(), accelRaw_[0],
             accelRaw_[1], accelRaw_[2]);

    return ESP_OK;
}

uint8_t ADXL345::getID(void) const {
    return kSensorID_;
}

uint8_t ADXL345::readDeviceID(void) const {
    uint8_t value = 0;
    readRegister(kDevIdReg, &value);
    return value;
}

esp_err_t ADXL345::configureActivityControl(ActivityControl config) const {
    return writeRegister(kActInactCtlReg, static_cast<uint8_t>(config));
}

esp_err_t ADXL345::configureActivityThreshold(uint8_t threshold) const {
    return writeRegister(kThreshActReg, threshold);
}

esp_err_t ADXL345::configureInactivityThreshold(uint8_t threshold) const {
    return writeRegister(kThreshInactReg, threshold);
}

esp_err_t ADXL345::configureInactivityTime(uint8_t time) const {
    return writeRegister(kTimeInactReg, time);
}

esp_err_t ADXL345::configurePowerControl(PowerControlBits config) const {
    return writeRegister(kPowerCtlReg, static_cast<uint8_t>(config));
}

esp_err_t ADXL345::clearPowerControlBits(PowerControlBits bits) const {
    uint8_t regValue = 0;

    esp_err_t err = readRegister(kPowerCtlReg, &regValue);
    if (err) {
        return err;
    }

    regValue &= ~(static_cast<uint8_t>(bits));
    return writeRegister(kPowerCtlReg, regValue);
}

esp_err_t ADXL345::configureOutPutDataRate(OutputDataRate rate) const {
    return writeRegister(kBwRateReg, static_cast<uint8_t>(rate));
}

esp_err_t ADXL345::configureOutPutDataRate(OutputDataRate rate,  // TODO
                                           PowerMode mode) const {
    uint8_t registerVal = 0;
    registerVal = (static_cast<uint8_t>(mode) << 4) | static_cast<uint8_t>(rate);
    return writeRegister(kBwRateReg, registerVal);
}

esp_err_t ADXL345::setFullResolution(void) {
    uint8_t registerVal = 0;
    esp_err_t err = readRegister(kDataFormatReg, &registerVal);
    registerVal = registerVal | 1 << 3;
    err = writeRegister(kDataFormatReg, registerVal);
    return err;
}

esp_err_t ADXL345::configureAccelerationRange(AccelerationRange range) {
    uint8_t registerVal = 0;
    esp_err_t err = readRegister(kDataFormatReg, &registerVal);
    registerVal = registerVal | static_cast<uint8_t>(range);
    err = writeRegister(kDataFormatReg, registerVal);
    return err;
}

esp_err_t ADXL345::configureFIFO(FifoMode mode, uint8_t samples, Interrupt intNum) const {
    uint8_t registerValue = 0;
    esp_err_t err = readRegister(kFifoCtlReg, &registerValue);
    if (err) {
        return err;
    }

    samples = samples & 0b11111;
    registerValue = static_cast<uint8_t>(mode) << 6;

    if (mode == FifoMode::kTrigger) {
        registerValue |= static_cast<uint8_t>(intNum) << 5;
    }

    registerValue |= samples;
    err = writeRegister(kFifoCtlReg, registerValue);
    return err;
}

esp_err_t ADXL345::enableMeasure(void) const {
    return enableMeasureBit();
}

esp_err_t ADXL345::disableMeasure(void) const {
    return disableMeasureBit();
}

bool ADXL345::checkDataReady(void) const {
    uint8_t regValue = 0;
    readRegister(kIntSourceReg, &regValue);
    return regValue & static_cast<uint8_t>(InterruptFlags::kDataReady);
}

uint8_t ADXL345::checkFifoStatus(void) const {
    uint8_t regValue = 0;
    readRegister(kFifoStatusReg, &regValue);
    return regValue;
}

esp_err_t ADXL345::calibrate(void) {
    esp_err_t err = enableMeasureBit();
    if (err) {
        return err;
    }

    return calibrateOffsets();
}

void ADXL345::getRawData(int16_t* buffer) const {
    buffer[0] = accelRaw_[0];
    buffer[1] = accelRaw_[1];
    buffer[2] = accelRaw_[2];
}

esp_err_t ADXL345::readRegister(uint8_t regAddr, uint8_t* value) const {
    return i2c_.read(slaveAddress_, regAddr, value, (uint8_t)1);
}

esp_err_t ADXL345::writeRegister(uint8_t regAddr, uint8_t value) const {
    return i2c_.write(slaveAddress_, regAddr, &value, (uint8_t)1);
}

esp_err_t ADXL345::enableMeasureBit(void) const {
    uint8_t regValue = 0;

    esp_err_t err = readRegister(kPowerCtlReg, &regValue);
    if (err) {
        return err;
    }

    regValue |= static_cast<uint8_t>(PowerControlBits::kMeasure);

    return writeRegister(kPowerCtlReg, regValue);
}

esp_err_t ADXL345::disableMeasureBit(void) const {
    uint8_t regValue = 0;

    esp_err_t err = readRegister(kPowerCtlReg, &regValue);
    if (err) {
        return err;
    }

    regValue &= ~static_cast<uint8_t>(PowerControlBits::kMeasure);

    return writeRegister(kPowerCtlReg, regValue);
}

esp_err_t ADXL345::collectSamples(int16_t* avgArray) {
    int32_t sum[3] = {0};

    const uint8_t kSamplesCount = 16;
    for (int i = 0; i < kSamplesCount; i++) {
        while (!checkDataReady());  // Wait until data is ready

        esp_err_t err = measure();
        if (err) {
            return err;
        }

        sum[0] += accelRaw_[0];  // x
        sum[1] += accelRaw_[1];  // y
        sum[2] += accelRaw_[2];  // z
    }

    avgArray[0] = sum[0] >> 4;  // Division by 16
    avgArray[1] = sum[1] >> 4;
    avgArray[2] = sum[2] >> 4;

    return ESP_OK;
}

esp_err_t ADXL345::readCurrentOffsets(int8_t* offsetArray) {
    uint8_t rawData[3] = {0};

    esp_err_t err = i2c_.read(slaveAddress_, kOfsXReg, rawData, 3);
    if (err) {
        return err;
    }

    offsetArray[0] = static_cast<int8_t>(rawData[0]);
    offsetArray[1] = static_cast<int8_t>(rawData[1]);
    offsetArray[2] = static_cast<int8_t>(rawData[2]);

    return ESP_OK;
}

esp_err_t ADXL345::updateOffsets(int16_t* avgArray) {
    int8_t offsets[3] = {0};

    esp_err_t err = readCurrentOffsets(offsets);
    if (err) {
        return err;
    }

    offsets[0] -= avgArray[0] >> 2;  // Division by 4
    offsets[1] -= avgArray[1] >> 2;
    offsets[2] -= avgArray[2] >> 2;

    return i2c_.write(slaveAddress_, kOfsXReg, reinterpret_cast<uint8_t*>(offsets), 3);
}

esp_err_t ADXL345::calibrateOffsets(void) {
    const uint8_t kConvergenceThreshold = 2;
    int16_t avg[3] = {0};

    const uint8_t kMaxTries = 100;
    uint8_t counter = 0;
    do {
        esp_err_t err = collectSamples(avg);
        if (err) {
            return err;
        }

        err = updateOffsets(avg);
        if (err) {
            return err;
        }

        ++counter;
        if (counter > kMaxTries) {
            return ESP_ERR_TIMEOUT;
        }

    } while (abs(avg[0]) > kConvergenceThreshold || abs(avg[1]) > kConvergenceThreshold ||
             abs(avg[2]) > kConvergenceThreshold);

    return ESP_OK;
}

esp_err_t ADXL345::assignInterrupts(InterruptFlags flags, Interrupt intNum) const {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kIntMapReg, &regValue);
    if (err) {
        return err;
    }

    if (intNum == Interrupt::INT2) {
        regValue |= static_cast<uint8_t>(flags);  // Set bits for INT2
    } else {
        regValue &= ~static_cast<uint8_t>(flags);  // Clear bits for INT1
    }

    err = writeRegister(kIntMapReg, regValue);

    return err;
}

esp_err_t ADXL345::setInterruptActiveLevel(ActiveLevel level) const {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kDataFormatReg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kIntInvertBit = 0x20;
    if (level == kLow) {
        regValue |= kIntInvertBit;
    } else if (level == kHigh) {
        regValue &= ~kIntInvertBit;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return writeRegister(kDataFormatReg, regValue);
}

esp_err_t ADXL345::enableInterrupts(InterruptFlags flags) {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kIntEnableReg, &regValue);
    if (err) {
        return err;
    }

    regValue |= static_cast<uint8_t>(flags);
    err = writeRegister(kIntEnableReg, regValue);
    return err;
}

esp_err_t ADXL345::disableInterrupts(InterruptFlags flags) {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kIntEnableReg, &regValue);
    if (err) {
        return err;
    }

    regValue &= ~static_cast<uint8_t>(flags);
    return writeRegister(kIntMapReg, regValue);
}

esp_err_t ADXL345::configurePin(gpio_num_t pin, Interrupt intNum, ActiveLevel level,
                                bool mcuIntEnable) {
    gpio_config_t ioConf = {};
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = (1ULL << pin);

    switch (level) {
        case ActiveLevel::kHigh:
            ioConf.pull_up_en = GPIO_PULLUP_DISABLE;
            ioConf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            ioConf.intr_type = mcuIntEnable ? GPIO_INTR_POSEDGE : GPIO_INTR_DISABLE;
            break;
        case ActiveLevel::kLow:
            ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
            ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            ioConf.intr_type = mcuIntEnable ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = gpio_config(&ioConf);
    if (err != ESP_OK) {
        return err;
    }

    if (intNum == Interrupt::INT1) {
        int1Gpio_ = pin;
    } else if (intNum == Interrupt::INT2) {
        int2Gpio_ = pin;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return setInterruptActiveLevel(level);
}

esp_err_t ADXL345::initializeSharedISR(void) {
    if (isIsrServiceInstalled_) {
        return ESP_OK;
    }

    // Low priority, edge detection, iram allocated -> (check size)
    const int isrFlags = ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM;

    esp_err_t err = gpio_install_isr_service(isrFlags);
    if (err == ESP_OK) {
        isIsrServiceInstalled_ = true;
    }

    return err;
}

esp_err_t ADXL345::attachInterrupt(Interrupt intNum, adxl345Callback_t callback,
                                   void* context) {
    if (callback == nullptr || context == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initializeSharedISR();
    if (err) {
        return err;
    }

    if (intNum == Interrupt::INT1) {
        int1Callback_ = callback;
        int1Context_ = context;
    } else if (intNum == Interrupt::INT2) {
        int2Callback_ = callback;
        int2Context_ = context;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return err;
}

esp_err_t ADXL345::detachInterrupt(Interrupt intNum) {
    esp_err_t err = ESP_OK;

    if (intNum == Interrupt::INT1) {
        int1Callback_ = nullptr;
        int1Context_ = nullptr;
    } else if (intNum == Interrupt::INT2) {
        int2Callback_ = nullptr;
        int2Context_ = nullptr;
    } else {
        return ESP_ERR_INVALID_ARG;  // Invalid interrupt type
    }

    return err;
}

esp_err_t ADXL345::enablePinInterrupt(Interrupt intNum) {
    esp_err_t err = ESP_OK;
    if (intNum == Interrupt::INT1) {
        err = gpio_isr_handler_add(int1Gpio_, handleINT1, int1Context_);
    } else if (intNum == Interrupt::INT2) {
        err = gpio_isr_handler_add(int2Gpio_, handleINT2, int2Context_);
    } else {
        return ESP_ERR_INVALID_ARG;  // Invalid interrupt type
    }

    return err;
}

esp_err_t ADXL345::disablePinInterrupt(Interrupt intNum) {
    esp_err_t err = ESP_OK;
    if (intNum == Interrupt::INT1) {
        err = gpio_isr_handler_remove(int1Gpio_);
    } else if (intNum == Interrupt::INT2) {
        err = gpio_isr_handler_remove(int2Gpio_);
    } else {
        return ESP_ERR_INVALID_ARG;  // Invalid interrupt type
    }

    return err;
}

void IRAM_ATTR ADXL345::handleINT1(void* arg) {
    ADXL345* sensor = static_cast<ADXL345*>(arg);
    if (sensor->int1Callback_) {
        sensor->int1Callback_(sensor->int1Context_);
    }
}

void IRAM_ATTR ADXL345::handleINT2(void* arg) {
    ADXL345* sensor = static_cast<ADXL345*>(arg);
    if (sensor->int2Callback_) {
        sensor->int2Callback_(sensor->int2Context_);
    }
}

esp_err_t ADXL345::getInterruptReason(InterruptFlags& flags) const {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kIntSourceReg, &regValue);
    if (err) {
        return err;
    }

    flags = static_cast<InterruptFlags>(regValue);
    return ESP_OK;
}

bool ADXL345::isIntFlagUp(InterruptFlags flag) const {
    InterruptFlags flags = static_cast<InterruptFlags>(0);
    if (getInterruptReason(flags) != ESP_OK) {
        return false;
    }
    return static_cast<bool>(flags & flag);
}
