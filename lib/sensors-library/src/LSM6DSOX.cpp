/*******************************************************************************
 *******************************************************************************
 * @file LSM6DSOX.cpp
 * @brief Contains the definitions of the LSM6DSOX class methods.
 *
 * This sensor has 3-axis accelerometer and 3-axis gyroscope with advanced
 * features including FIFO, interrupts, and embedded functions.
 *
 * @version v0.1.0
 * @date 2025-11-18
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include <cmath>

#include "LSM6DSOX.hpp"
#include "esp_system.h"

bool LSM6DSOX::isIsrServiceInstalled_ = false;

LSM6DSOX::InterruptFlags operator|(LSM6DSOX::InterruptFlags a,
                                   LSM6DSOX::InterruptFlags b) {
    return static_cast<LSM6DSOX::InterruptFlags>(static_cast<uint8_t>(a) |
                                                 static_cast<uint8_t>(b));
}

LSM6DSOX::InterruptFlags operator&(LSM6DSOX::InterruptFlags a,
                                   LSM6DSOX::InterruptFlags b) {
    return static_cast<LSM6DSOX::InterruptFlags>(static_cast<uint8_t>(a) &
                                                 static_cast<uint8_t>(b));
}

LSM6DSOX::InterruptFlags operator~(LSM6DSOX::InterruptFlags a) {
    return static_cast<LSM6DSOX::InterruptFlags>(~static_cast<uint8_t>(a));
}

LSM6DSOX::LSM6DSOX(I2C& i2cInstance, uint8_t slaveAddress)
    : commInterface_(CommInterface::kI2C),
      i2c_(&i2cInstance),
      slaveAddress_(slaveAddress) {
}

LSM6DSOX::LSM6DSOX(SPI& spiInstance, spi_device_handle_t* deviceHandle)
    : commInterface_(CommInterface::kSPI),
      spi_(&spiInstance),
      spiDeviceHandle_(deviceHandle) {
}

LSM6DSOX::~LSM6DSOX() {
}

esp_err_t LSM6DSOX::init(void) {
    uint8_t id = readDeviceID();
    if (id != kDeviceId) {
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = softwareReset();
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t kSwResetBit = 0x01;  // Wait for reset to complete
    uint8_t rstCheck = 1;
    int timeout = 100;
    while ((rstCheck & kSwResetBit) && timeout > 0) {
        readRegister(kCtrl3CReg, &rstCheck);
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
    }

    if (timeout == 0) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t ctrl9 = 0;  // Disable I3C interface
    err = readRegister(kCtrl9XlReg, &ctrl9);
    if (err != ESP_OK) return err;

    const uint8_t kI3cDisableBit = 0x02;
    ctrl9 |= kI3cDisableBit;
    err = writeRegister(kCtrl9XlReg, ctrl9);
    if (err != ESP_OK) return err;

    // Enable Block Data Update (BDU) and Auto-Increment (IF_INC)
    const uint8_t kBduIfIncBit = 0x04;
    const uint8_t kBduBit = 0x40;
    const uint8_t kBduIncValue = kBduBit | kBduIfIncBit;
    err = writeRegister(kCtrl3CReg, kBduIncValue);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t LSM6DSOX::measure(void) {
    const uint8_t kDataLength = 12;
    uint8_t rawData[kDataLength] = {0};

    // Read gyroscope and accelerometer data
    esp_err_t err = readRegister(kOutXLGReg, rawData, kDataLength);
    if (err) {
        return err;
    }

    for (uint8_t i = 0; i < 3; i++) {  // Gyro X, Y, Z
        uint16_t raw = (static_cast<uint16_t>(rawData[(2 * i) + 1]) << 8) |
                       static_cast<uint16_t>(rawData[2 * i]);
        gyroRaw_[i] = static_cast<int16_t>(raw);
    }

    for (uint8_t i = 0; i < 3; i++) {  // Accel X, Y, Z
        uint16_t raw = (static_cast<uint16_t>(rawData[6 + (2 * i) + 1]) << 8) |
                       static_cast<uint16_t>(rawData[6 + (2 * i)]);
        accelRaw_[i] = static_cast<int16_t>(raw);
    }

    return ESP_OK;
}

esp_err_t LSM6DSOX::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 256,
             "ID: %X, ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d, temp: %.2f°C",
             getID(), accelRaw_[0], accelRaw_[1], accelRaw_[2], gyroRaw_[0], gyroRaw_[1],
             gyroRaw_[2], getTemperatureCelsius());

    return ESP_OK;
}

uint8_t LSM6DSOX::getID(void) const {
    return kSensorID_;
}

uint8_t LSM6DSOX::readDeviceID(void) const {
    uint8_t value = 0;
    readRegister(kWhoAmIReg, &value);
    return value;
}

esp_err_t LSM6DSOX::configureAccelDataRate(AccelOutputDataRate rate) const {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kCtrl1XlReg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kOdrMask = 0xF0;
    regValue = (regValue & ~kOdrMask) | (static_cast<uint8_t>(rate) << 4);
    return writeRegister(kCtrl1XlReg, regValue);
}

esp_err_t LSM6DSOX::configureGyroDataRate(GyroOutputDataRate rate) const {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kCtrl2GReg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kOdrMask = 0xF0;
    regValue = (regValue & ~kOdrMask) | (static_cast<uint8_t>(rate) << 4);
    return writeRegister(kCtrl2GReg, regValue);
}

esp_err_t LSM6DSOX::configureAccelFullScale(AccelFullScale scale) {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kCtrl1XlReg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kFsMask = 0x0C;
    regValue = (regValue & ~kFsMask) | (static_cast<uint8_t>(scale) << 2);
    err = writeRegister(kCtrl1XlReg, regValue);

    if (err == ESP_OK) {
        accelScale_ = scale;
    }

    return err;
}

esp_err_t LSM6DSOX::configureGyroFullScale(GyroFullScale scale) {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kCtrl2GReg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kFsMask = 0x0C;
    regValue = (regValue & ~kFsMask) | (static_cast<uint8_t>(scale) << 2);
    err = writeRegister(kCtrl2GReg, regValue);

    if (err == ESP_OK) {
        gyroScale_ = scale;
    }

    return err;
}

esp_err_t LSM6DSOX::configureFIFO(FifoMode mode, uint16_t watermark,
                                  BatchDataRate accelRate, BatchDataRate gyroRate) const {
    uint8_t bdr = (static_cast<uint8_t>(gyroRate) << 4) | static_cast<uint8_t>(accelRate);
    esp_err_t err = writeRegister(kFifoCtrl3Reg, bdr);
    if (err) return err;

    err = writeRegister(kFifoCtrl1Reg, watermark & 0xFF);
    if (err) return err;

    uint8_t fifoCtrl2 = 0;
    err = readRegister(kFifoCtrl2Reg, &fifoCtrl2);
    if (err) return err;

    const uint8_t kWtmMsbMask = 0x01;
    fifoCtrl2 &= ~kWtmMsbMask;
    fifoCtrl2 |= (watermark >> 8) & kWtmMsbMask;

    err = writeRegister(kFifoCtrl2Reg, fifoCtrl2);
    if (err) return err;

    uint8_t fifoCtrl4 = 0;
    err = readRegister(kFifoCtrl4Reg, &fifoCtrl4);
    if (err) return err;

    const uint8_t kFifoModeMask = 0x07;
    fifoCtrl4 &= ~kFifoModeMask;
    fifoCtrl4 |= static_cast<uint8_t>(mode) & kFifoModeMask;

    return writeRegister(kFifoCtrl4Reg, fifoCtrl4);
}

esp_err_t LSM6DSOX::enableAccel(AccelOutputDataRate rate) const {
    return configureAccelDataRate(rate);
}

esp_err_t LSM6DSOX::enableGyro(GyroOutputDataRate rate) const {
    return configureGyroDataRate(rate);
}

esp_err_t LSM6DSOX::disableAccel(void) const {
    return configureAccelDataRate(AccelOutputDataRate::kPowerDown);
}

esp_err_t LSM6DSOX::disableGyro(void) const {
    return configureGyroDataRate(GyroOutputDataRate::kPowerDown);
}

bool LSM6DSOX::checkAccelDataReady(void) const {
    uint8_t status = 0;
    readRegister(kStatusReg, &status);
    const uint8_t kXldaBit = 0x01;
    return (status & kXldaBit) != 0;  // XLDA bit
}

bool LSM6DSOX::checkGyroDataReady(void) const {
    uint8_t status = 0;
    readRegister(kStatusReg, &status);
    const uint8_t kGdaBit = 0x02;
    return (status & kGdaBit) != 0;  // GDA bit
}

esp_err_t LSM6DSOX::measureTemperature(void) {
    uint8_t rawData[2] = {0};

    esp_err_t err = readRegister(kOutTempLReg, rawData, 2);
    if (err) {
        return err;
    }

    tempRaw_ = (static_cast<int16_t>(rawData[1]) << 8) | rawData[0];

    return ESP_OK;
}

int16_t LSM6DSOX::readTemperature(void) {
    measureTemperature();
    return tempRaw_;
}

void LSM6DSOX::getAccelRawData(int16_t* buffer) const {
    buffer[0] = accelRaw_[0];
    buffer[1] = accelRaw_[1];
    buffer[2] = accelRaw_[2];
}

void LSM6DSOX::getGyroRawData(int16_t* buffer) const {
    buffer[0] = gyroRaw_[0];
    buffer[1] = gyroRaw_[1];
    buffer[2] = gyroRaw_[2];
}

int16_t LSM6DSOX::getTemperatureRaw(void) const {
    return tempRaw_;
}

float LSM6DSOX::getTemperatureCelsius(void) const {
    // LSM6DSOX temperature formula: T(°C) = (temp_raw / 256) + 25
    return (tempRaw_ / 256.0f) + 25.0f;
}

float LSM6DSOX::getPitch(void) const {
    // Pitch (around Y-axis) = atan2(-Ax, sqrt(Ay^2 + Az^2))
    float ax = static_cast<float>(accelRaw_[0]);
    float ay = static_cast<float>(accelRaw_[1]);
    float az = static_cast<float>(accelRaw_[2]);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    return atan2(-ax, sqrt(ay * ay + az * az)) * (180.0f / M_PI);
}

float LSM6DSOX::getRoll(void) const {
    // Roll (around X-axis)
    // phi = atan2(Ay, sqrt(Ax^2 + Az^2))
    float ax = static_cast<float>(accelRaw_[0]);
    float ay = static_cast<float>(accelRaw_[1]);
    float az = static_cast<float>(accelRaw_[2]);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    return atan2(ay, sqrt(ax * ax + az * az)) * (180.0f / M_PI);
}

esp_err_t LSM6DSOX::enableInterrupts(InterruptFlags flags, Interrupt intNum) {
    uint8_t regAddr = (intNum == Interrupt::kInt1) ? kInt1CtrlReg : kInt2CtrlReg;
    uint8_t regValue = 0;

    esp_err_t err = readRegister(regAddr, &regValue);
    if (err) {
        return err;
    }

    regValue |= static_cast<uint8_t>(flags);
    return writeRegister(regAddr, regValue);
}

esp_err_t LSM6DSOX::disableInterrupts(InterruptFlags flags, Interrupt intNum) {
    uint8_t regAddr = (intNum == Interrupt::kInt1) ? kInt1CtrlReg : kInt2CtrlReg;
    uint8_t regValue = 0;

    esp_err_t err = readRegister(regAddr, &regValue);
    if (err) {
        return err;
    }

    regValue &= ~static_cast<uint8_t>(flags);
    return writeRegister(regAddr, regValue);
}

esp_err_t LSM6DSOX::configurePin(gpio_num_t pin, Interrupt intNum, bool activeHigh,
                                 bool mcuIntEnable) {
    gpio_config_t ioConf = {};
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = (1ULL << pin);

    if (activeHigh) {
        ioConf.pull_up_en = GPIO_PULLUP_DISABLE;
        ioConf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        ioConf.intr_type = mcuIntEnable ? GPIO_INTR_POSEDGE : GPIO_INTR_DISABLE;
    } else {
        ioConf.pull_up_en = GPIO_PULLUP_ENABLE;
        ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        ioConf.intr_type = mcuIntEnable ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE;
    }

    esp_err_t err = gpio_config(&ioConf);
    if (err != ESP_OK) {
        return err;
    }

    if (intNum == Interrupt::kInt1) {
        int1Gpio_ = pin;
    } else if (intNum == Interrupt::kInt2) {
        int2Gpio_ = pin;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure interrupt pin polarity in CTRL3_C register
    uint8_t ctrl3Value = 0;
    err = readRegister(kCtrl3CReg, &ctrl3Value);
    if (err) {
        return err;
    }

    const uint8_t kHLactiveBit = 0x20;
    if (activeHigh) {
        ctrl3Value &= ~kHLactiveBit;  // Clear H_LACTIVE bit (active high)
    } else {
        ctrl3Value |= kHLactiveBit;  // Set H_LACTIVE bit (active low)
    }

    return writeRegister(kCtrl3CReg, ctrl3Value);
}

esp_err_t LSM6DSOX::attachInterrupt(Interrupt intNum, lsm6dsoxCallback_t callback,
                                    void* context) {
    if (callback == nullptr || context == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initializeSharedISR();
    if (err) {
        return err;
    }

    if (intNum == Interrupt::kInt1) {
        int1Callback_ = callback;
        int1Context_ = context;
    } else if (intNum == Interrupt::kInt2) {
        int2Callback_ = callback;
        int2Context_ = context;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t LSM6DSOX::detachInterrupt(Interrupt intNum) {
    if (intNum == Interrupt::kInt1) {
        int1Callback_ = nullptr;
        int1Context_ = nullptr;
    } else if (intNum == Interrupt::kInt2) {
        int2Callback_ = nullptr;
        int2Context_ = nullptr;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t LSM6DSOX::enablePinInterrupt(Interrupt intNum) {
    if (intNum == Interrupt::kInt1) {
        return gpio_isr_handler_add(int1Gpio_, handleINT1, int1Context_);
    } else if (intNum == Interrupt::kInt2) {
        return gpio_isr_handler_add(int2Gpio_, handleINT2, int2Context_);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t LSM6DSOX::disablePinInterrupt(Interrupt intNum) {
    if (intNum == Interrupt::kInt1) {
        return gpio_isr_handler_remove(int1Gpio_);
    } else if (intNum == Interrupt::kInt2) {
        return gpio_isr_handler_remove(int2Gpio_);
    } else {
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t LSM6DSOX::setInterruptLatching(bool enable) {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kTapCfg0Reg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kLirBit = 0x80;  // Bit 7 is LIR
    if (enable) {
        regValue |= kLirBit;
    } else {
        regValue &= ~kLirBit;
    }

    return writeRegister(kTapCfg0Reg, regValue);
}

esp_err_t LSM6DSOX::configureWakeUp(uint8_t threshold, uint8_t duration) {
    // Set Wake-up Threshold (WAKE_UP_THS)
    uint8_t wakeUpThs = 0;
    esp_err_t err = readRegister(kWakeUpThsReg, &wakeUpThs);
    if (err) return err;

    const uint8_t kWakeUpThsMask = 0x3F;
    wakeUpThs &= ~kWakeUpThsMask;
    wakeUpThs |= (threshold & kWakeUpThsMask);

    err = writeRegister(kWakeUpThsReg, wakeUpThs);
    if (err) return err;

    // Set Wake-up Duration (WAKE_UP_DUR)
    uint8_t wakeUpDur = 0;
    err = readRegister(kWakeUpDurReg, &wakeUpDur);
    if (err) return err;

    const uint8_t kWakeUpDurValMask = 0x03;
    const uint8_t kWakeUpDurRegMask = 0x60;  // Bits [6:5]

    wakeUpDur &= ~kWakeUpDurRegMask;
    wakeUpDur |= ((duration & kWakeUpDurValMask) << 5);

    err = writeRegister(kWakeUpDurReg, wakeUpDur);
    if (err) return err;

    // Enable Wake-up detection in TAP_CFG2 (INTERRUPTS_ENABLE)
    uint8_t tapCfg2 = 0;
    err = readRegister(kTapCfg2Reg, &tapCfg2);
    if (err) return err;

    const uint8_t kInterruptsEnableBit = 0x80;
    tapCfg2 |= kInterruptsEnableBit;  // Set INTERRUPTS_ENABLE
    return writeRegister(kTapCfg2Reg, tapCfg2);
}

esp_err_t LSM6DSOX::enableWakeUpInterrupt(bool enable, Interrupt pin) {
    uint8_t regAddr = (pin == Interrupt::kInt1) ? kMd1CfgReg : kMd2CfgReg;
    uint8_t mdCfg = 0;
    esp_err_t err = readRegister(regAddr, &mdCfg);
    if (err) return err;

    const uint8_t kIntWuBit = 0x20;  // Bit 5 is INT1_WU or INT2_WU
    if (enable) {
        mdCfg |= kIntWuBit;
    } else {
        mdCfg &= ~kIntWuBit;
    }
    return writeRegister(regAddr, mdCfg);
}

bool LSM6DSOX::checkWakeUp(void) const {
    WakeUpSrcFlags src = getWakeUpSrc();
    return (src & WakeUpSrcFlags::kWakeUpStatus) != WakeUpSrcFlags::kNone;
}

esp_err_t LSM6DSOX::softwareReset(void) const {
    uint8_t regValue = 0;
    esp_err_t err = readRegister(kCtrl3CReg, &regValue);
    if (err) {
        return err;
    }

    const uint8_t kSwResetBit = 0x01;
    regValue |= kSwResetBit;  // Set SW_RESET bit
    return writeRegister(kCtrl3CReg, regValue);
}

uint16_t LSM6DSOX::getFifoStatus(void) const {
    uint8_t status[2] = {0};
    readRegister(kFifoStatus1Reg, status, 2);

    const uint8_t kFifoStatusDiffMask = 0x03;  // 2 LSB bits
    return (static_cast<uint16_t>(status[1] & kFifoStatusDiffMask) << 8) | status[0];
}

esp_err_t LSM6DSOX::readFifoWord(FifoData* fifoData) const {
    if (fifoData == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t rawData[7] = {0};  // Burst read 7 bytes (1 Tag + 6 Data)
    esp_err_t err = readRegister(kFifoDataOutTagReg, rawData, 7);
    if (err != ESP_OK) {
        return err;
    }

    // Parse tag and data (1 FIFO word)
    fifoData->tag = static_cast<FifoTag>(rawData[0] >> 3);
    for (int i = 0; i < 6; i++) {
        fifoData->data[i] = rawData[i + 1];
    }

    return ESP_OK;
}

esp_err_t LSM6DSOX::readFifoWords(FifoData* fifoData, uint16_t numWords) const {
    if (fifoData == nullptr || numWords == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint16_t i = 0; i < numWords; i++) {
        esp_err_t err = readFifoWord(&fifoData[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

bool LSM6DSOX::parseFifoAccelData(const FifoData* fifoData, int16_t* accelData) const {
    if (fifoData == nullptr || accelData == nullptr) {
        return false;
    }

    // Check if tag indicates accelerometer data
    if (fifoData->tag != FifoTag::kAccelNC && fifoData->tag != FifoTag::kAccelNC_T_1 &&
        fifoData->tag != FifoTag::kAccelNC_T_2) {
        return false;
    }

    accelData[0] = static_cast<int16_t>((fifoData->data[1] << 8) | fifoData->data[0]);
    accelData[1] = static_cast<int16_t>((fifoData->data[3] << 8) | fifoData->data[2]);
    accelData[2] = static_cast<int16_t>((fifoData->data[5] << 8) | fifoData->data[4]);

    return true;
}

bool LSM6DSOX::parseFifoGyroData(const FifoData* fifoData, int16_t* gyroData) const {
    if (fifoData == nullptr || gyroData == nullptr) {
        return false;
    }

    // Check if tag indicates gyroscope data
    if (fifoData->tag != FifoTag::kGyroNC && fifoData->tag != FifoTag::kGyro_NC_T_1 &&
        fifoData->tag != FifoTag::kGyro_NC_T_2) {
        return false;
    }

    gyroData[0] = static_cast<int16_t>((fifoData->data[1] << 8) | fifoData->data[0]);
    gyroData[1] = static_cast<int16_t>((fifoData->data[3] << 8) | fifoData->data[2]);
    gyroData[2] = static_cast<int16_t>((fifoData->data[5] << 8) | fifoData->data[4]);

    return true;
}

esp_err_t LSM6DSOX::readRegister(uint8_t regAddr, uint8_t* value, size_t len) const {
    if (commInterface_ == CommInterface::kI2C) {
        return readRegisterI2C(regAddr, value, len);
    } else {
        return readRegisterSPI(regAddr, value, len);
    }
}

esp_err_t LSM6DSOX::writeRegister(uint8_t regAddr, uint8_t value) const {
    if (commInterface_ == CommInterface::kI2C) {
        return writeRegisterI2C(regAddr, value);
    } else {
        return writeRegisterSPI(regAddr, value);
    }
}

esp_err_t LSM6DSOX::readRegisterI2C(uint8_t regAddr, uint8_t* data, size_t len) const {
    return i2c_->read(slaveAddress_, regAddr, data, len);
}

esp_err_t LSM6DSOX::writeRegisterI2C(uint8_t regAddr, uint8_t value) const {
    return i2c_->write(slaveAddress_, regAddr, &value, 1);
}

esp_err_t LSM6DSOX::readRegisterSPI(uint8_t regAddr, uint8_t* data, size_t len) const {
    // LSM6DSOX SPI read: MSB = 1 for read
    uint8_t txBuffer[len + 1];
    uint8_t rxBuffer[len + 1];

    const uint8_t kSpiReadBit = 0x80;
    txBuffer[0] = regAddr | kSpiReadBit;  // Set MSB for read operation
    for (size_t i = 1; i <= len; i++) {
        txBuffer[i] = 0x00;  // Dummy bytes
    }

    esp_err_t err = spi_->transfer(spiDeviceHandle_, txBuffer, rxBuffer, len + 1);
    if (err == ESP_OK) {
        // Copy received data (skip first byte which is dummy)
        for (size_t i = 0; i < len; i++) {
            data[i] = rxBuffer[i + 1];
        }
    }

    return err;
}

esp_err_t LSM6DSOX::writeRegisterSPI(uint8_t regAddr, uint8_t value) const {
    // LSM6DSOX SPI write: MSB = 0 for write
    uint8_t txBuffer[2];
    const uint8_t kSpiWriteMask = 0x7F;
    txBuffer[0] = regAddr & kSpiWriteMask;  // Clear MSB for write operation
    txBuffer[1] = value;

    return spi_->write(spiDeviceHandle_, txBuffer, 2);
}

esp_err_t LSM6DSOX::initializeSharedISR(void) {
    if (isIsrServiceInstalled_) {
        return ESP_OK;
    }

    const int isrFlags = 0;  // Evaluate if this should be lv 1+
    esp_err_t err = gpio_install_isr_service(isrFlags);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        isIsrServiceInstalled_ = true;
        return ESP_OK;
    }

    return err;
}

void IRAM_ATTR LSM6DSOX::handleINT1(void* arg) {
    LSM6DSOX* sensor = static_cast<LSM6DSOX*>(arg);
    if (sensor->int1Callback_) {
        sensor->int1Callback_(sensor->int1Context_);
    }
}

void IRAM_ATTR LSM6DSOX::handleINT2(void* arg) {
    LSM6DSOX* sensor = static_cast<LSM6DSOX*>(arg);
    if (sensor->int2Callback_) {
        sensor->int2Callback_(sensor->int2Context_);
    }
}

LSM6DSOX::WakeUpSrcFlags operator|(LSM6DSOX::WakeUpSrcFlags a,
                                   LSM6DSOX::WakeUpSrcFlags b) {
    return static_cast<LSM6DSOX::WakeUpSrcFlags>(static_cast<uint8_t>(a) |
                                                 static_cast<uint8_t>(b));
}

LSM6DSOX::WakeUpSrcFlags operator&(LSM6DSOX::WakeUpSrcFlags a,
                                   LSM6DSOX::WakeUpSrcFlags b) {
    return static_cast<LSM6DSOX::WakeUpSrcFlags>(static_cast<uint8_t>(a) &
                                                 static_cast<uint8_t>(b));
}

LSM6DSOX::WakeUpSrcFlags operator~(LSM6DSOX::WakeUpSrcFlags a) {
    return static_cast<LSM6DSOX::WakeUpSrcFlags>(~static_cast<uint8_t>(a));
}

LSM6DSOX::AllIntSrcFlags operator|(LSM6DSOX::AllIntSrcFlags a,
                                   LSM6DSOX::AllIntSrcFlags b) {
    return static_cast<LSM6DSOX::AllIntSrcFlags>(static_cast<uint8_t>(a) |
                                                 static_cast<uint8_t>(b));
}

LSM6DSOX::AllIntSrcFlags operator&(LSM6DSOX::AllIntSrcFlags a,
                                   LSM6DSOX::AllIntSrcFlags b) {
    return static_cast<LSM6DSOX::AllIntSrcFlags>(static_cast<uint8_t>(a) &
                                                 static_cast<uint8_t>(b));
}

LSM6DSOX::AllIntSrcFlags operator~(LSM6DSOX::AllIntSrcFlags a) {
    return static_cast<LSM6DSOX::AllIntSrcFlags>(~static_cast<uint8_t>(a));
}

esp_err_t LSM6DSOX::configure6DOrientation(SixDThreshold threshold) {
    uint8_t thsVal = static_cast<uint8_t>(threshold);

    uint8_t tapThs6d = 0;
    esp_err_t err = readRegister(kTapThs6dReg, &tapThs6d);
    if (err) return err;

    const uint8_t kSixdThsMask = 0x60;  // Bits [6:5]
    tapThs6d &= ~kSixdThsMask;
    tapThs6d |= (thsVal << 5);

    err = writeRegister(kTapThs6dReg, tapThs6d);
    if (err) return err;

    // This bit may be needed for the correct working of 6D functionality
    uint8_t tapCfg2 = 0;
    err = readRegister(kTapCfg2Reg, &tapCfg2);
    if (err) return err;

    const uint8_t kInterruptsEnableBit = 0x80;
    tapCfg2 |= kInterruptsEnableBit;
    return writeRegister(kTapCfg2Reg, tapCfg2);
}

esp_err_t LSM6DSOX::enable6DInterrupt(bool enable, Interrupt pin) {
    uint8_t regAddr = (pin == Interrupt::kInt1) ? kMd1CfgReg : kMd2CfgReg;
    uint8_t mdCfg = 0;
    esp_err_t err = readRegister(regAddr, &mdCfg);
    if (err) return err;

    const uint8_t kInt6dBit = 0x04;  // Bit 2 is INT1_6D or INT2_6D
    if (enable) {
        mdCfg |= kInt6dBit;
    } else {
        mdCfg &= ~kInt6dBit;
    }
    return writeRegister(regAddr, mdCfg);
}

bool LSM6DSOX::check6DOrientation(uint8_t* position) const {
    uint8_t src = 0;
    esp_err_t err = readRegister(kD6dSrcReg, &src);
    if (err != ESP_OK) {
        return false;
    }

    // Bit 6: D6D_IA (Interrupt Active)
    const uint8_t kD6dIaBit = 0x40;
    bool active = (src & kD6dIaBit) != 0;

    if (active && position != nullptr) {
        // Decode position from bits [5:0]
        // ZH(5), ZL(4), YH(3), YL(2), XH(1), XL(0)
        const uint8_t kZhBit = 0x20;
        const uint8_t kZlBit = 0x10;
        const uint8_t kYhBit = 0x08;
        const uint8_t kYlBit = 0x04;
        const uint8_t kXhBit = 0x02;
        const uint8_t kXlBit = 0x01;

        if (src & kZhBit)
            *position = 5;  // ZH
        else if (src & kZlBit)
            *position = 4;  // ZL
        else if (src & kYhBit)
            *position = 3;  // YH
        else if (src & kYlBit)
            *position = 2;  // YL
        else if (src & kXhBit)
            *position = 1;  // XH
        else if (src & kXlBit)
            *position = 0;  // XL
    }

    return active;
}

esp_err_t LSM6DSOX::configureTiltDetection(void) {
    esp_err_t err = ESP_OK;

    const uint8_t kFuncCfgEnBit = 0x80;
    err |= writeRegister(kFuncCfgAccessReg, kFuncCfgEnBit);

    const uint8_t kTiltEnBit = 0x10;
    err |= writeRegister(kEmbFuncEnAReg, kTiltEnBit);

    const uint8_t kInt1TiltBit = 0x10;
    err |= writeRegister(kEmbFuncInt1Reg, kInt1TiltBit);

    const uint8_t kEmbFuncLirBit = 0x80;
    err |= writeRegister(kPageRwReg, kEmbFuncLirBit);

    err |= writeRegister(kFuncCfgAccessReg, 0x00);

    return err;
}

esp_err_t LSM6DSOX::enableTiltInterrupt(bool enable, Interrupt pin) {
    uint8_t regAddr = (pin == Interrupt::kInt1) ? kMd1CfgReg : kMd2CfgReg;
    uint8_t mdCfg = 0;
    esp_err_t err = readRegister(regAddr, &mdCfg);
    if (err) return err;

    const uint8_t kIntEmbFuncBit = 0x02;  // Bit 1 is INT1_EMB_FUNC or INT2_EMB_FUNC
    if (enable) {
        mdCfg |= kIntEmbFuncBit;
    } else {
        mdCfg &= ~kIntEmbFuncBit;
    }
    return writeRegister(regAddr, mdCfg);
}

bool LSM6DSOX::checkTilt(void) const {
    uint8_t status = 0;
    esp_err_t err = readRegister(kEmbFuncStatusMainpageReg, &status);
    if (err != ESP_OK) {
        return false;
    }
    const uint8_t kIsTiltBit = 0x10;
    return (status & kIsTiltBit) != 0;
}

esp_err_t LSM6DSOX::enableAccelLowPowerMode(bool enable) {
    return configureAccelPowerMode(enable ? AccelPowerMode::kLowPowerNormal
                                          : AccelPowerMode::kHighPerformance);
}

esp_err_t LSM6DSOX::configureAccelPowerMode(AccelPowerMode mode) {
    uint8_t ctrl6 = 0;
    esp_err_t err = readRegister(kCtrl6CReg, &ctrl6);
    if (err) return err;

    const uint8_t kXlHmModeBit = 0x10;  // Bit 4: XL_HM_MODE
    // Note: XL_ULP_EN is typically Bit 5 in newer sensors (LSM6DSV), but Bit 5 is LVL2_EN
    // in standard LSM6DSOX. We will attempt to set Bit 5 for kUltraLowPower if requested,
    // assuming the user has compatible hardware.
    const uint8_t kXlUlpEnBit = 0x20;  // Bit 5

    // Reset bits
    ctrl6 &= ~(kXlHmModeBit | kXlUlpEnBit);

    switch (mode) {
        case AccelPowerMode::kHighPerformance:
            // Both 0
            break;
        case AccelPowerMode::kLowPowerNormal:
            ctrl6 |= kXlHmModeBit;
            break;
        case AccelPowerMode::kUltraLowPower:
            // Based on user provided table: XL_ULP_EN = 1, XL_HM_MODE = 0
            ctrl6 |= kXlUlpEnBit;
            break;
    }

    return writeRegister(kCtrl6CReg, ctrl6);
}

esp_err_t LSM6DSOX::configureGyroPowerMode(GyroPowerMode mode) {
    uint8_t ctrl7 = 0;
    esp_err_t err = readRegister(kCtrl7GReg, &ctrl7);
    if (err) return err;

    const uint8_t kGHmModeBit = 0x80;  // Bit 7: G_HM_MODE

    if (mode == GyroPowerMode::kLowPowerNormal) {
        ctrl7 |= kGHmModeBit;
    } else {
        ctrl7 &= ~kGHmModeBit;
    }

    return writeRegister(kCtrl7GReg, ctrl7);
}

esp_err_t LSM6DSOX::calibrate(CalibrationMode mode) {
    esp_err_t err = ESP_OK;

    uint8_t originalXlCtrl1 = 0;  // For current configuration (CTRL1_XL)
    err = readRegister(kCtrl1XlReg, &originalXlCtrl1);
    if (err) return err;

    uint8_t calibXlCtrl1 = (static_cast<uint8_t>(AccelOutputDataRate::k104Hz) << 4) |
                           (static_cast<uint8_t>(AccelFullScale::k2g) << 2);
    err = writeRegister(kCtrl1XlReg, calibXlCtrl1);
    if (err) return err;

    // datasheet recommends discarding first few samples
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t ctrl7 = 0;
    err = readRegister(kCtrl7GReg, &ctrl7);
    if (err) return err;

    const uint8_t kUsrOffOnOutBit = 0x02;
    if (!(ctrl7 & kUsrOffOnOutBit)) {
        ctrl7 |= kUsrOffOnOutBit;
        err = writeRegister(kCtrl7GReg, ctrl7);
        if (err) return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    const int kSamples = 64;
    int32_t sum[3] = {0};

    for (int i = 0; i < kSamples; i++) {
        err = measure();
        if (err) return err;

        sum[0] += accelRaw_[0];
        sum[1] += accelRaw_[1];
        sum[2] += accelRaw_[2];

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay for 104Hz ODR (~10ms)
    }

    int32_t avg[3];
    avg[0] = sum[0] / kSamples;
    avg[1] = sum[1] / kSamples;
    avg[2] = sum[2] / kSamples;

    // At 2g Full Scale: 1 LSB = 0.061 mg
    // Offset Register: 1 LSB = 2^-10 g = ~0.977 mg
    // Ratio = 0.061 / 0.977 ~= 1/16
    int8_t offsets[3] = {0};
    const int32_t k1gThreshold = 8192;  // ~0.5g at 2g scale
    const int32_t k1gTarget = 16384;    // 1g at 2g scale

    for (int i = 0; i < 3; i++) {
        int32_t target = 0;

        if (mode == CalibrationMode::kSmart) {
            if (avg[i] > k1gThreshold) {
                target = k1gTarget;
            } else if (avg[i] < -k1gThreshold) {
                target = -k1gTarget;
            }
        }

        int32_t error = avg[i] - target;

        // Offset: invert error and scale down by 16
        offsets[i] = static_cast<int8_t>(-(error / 16));
    }

    err |= writeRegister(kXOfsUsrReg, static_cast<uint8_t>(offsets[0]));
    err |= writeRegister(kYOfsUsrReg, static_cast<uint8_t>(offsets[1]));
    err |= writeRegister(kZOfsUsrReg, static_cast<uint8_t>(offsets[2]));

    // Restore original configuration
    err |= writeRegister(kCtrl1XlReg, originalXlCtrl1);

    return err;
}

LSM6DSOX::WakeUpSrcFlags LSM6DSOX::getWakeUpSrc(void) const {
    uint8_t src = 0;
    readRegister(kWakeUpSrcReg, &src);
    return static_cast<WakeUpSrcFlags>(src);
}

LSM6DSOX::AllIntSrcFlags LSM6DSOX::getAllIntSrc(void) const {
    uint8_t src = 0;
    readRegister(kAllIntSrcReg, &src);
    return static_cast<AllIntSrcFlags>(src);
}
