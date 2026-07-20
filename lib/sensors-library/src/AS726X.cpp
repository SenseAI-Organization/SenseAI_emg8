/*******************************************************************************
 * @file AS726X.cpp
 * @brief Contains the definitions of the AS726X class methods.
 *
 * @version v0.1.0
 * @date 2024-05-15
 * @author Sense-AI
 *******************************************************************************/

#include "AS726X.hpp"

AS726X::AS726X(I2C& i2c, uint8_t device_address)
    : i2c_(i2c), device_address_(device_address) {}


esp_err_t AS726X::writeRegister(uint8_t reg, uint8_t value) {
    return virtualWriteRegister(reg, value);
}


esp_err_t AS726X::readRegister(uint8_t reg, uint8_t& value) {
    return virtualReadRegister(reg, value);
}


bool AS726X::pollStatus(uint8_t bit_mask, bool expected_value) {
    uint8_t status;

    do {
        esp_err_t err = i2c_.read(device_address_, kStatusReg, &status, 1);
        ESP_ERROR_CHECK(err); // Maybe this would need a change in the future.

    } while ((status & bit_mask) != (expected_value ? bit_mask : 0));

    return true;
}


esp_err_t AS726X::virtualWriteRegister(uint8_t virtualRegAddr, uint8_t dataToWrite) {
    esp_err_t err = ESP_OK;

    if (!pollStatus(kTxValidBit, false)) {
        err = ESP_ERR_INVALID_RESPONSE;
        return err;
    }

    uint8_t temporalAddress = virtualRegAddr | 0x80;

    err = i2c_.write(device_address_, kWriteReg, &temporalAddress, 1);
    if (err != ESP_OK) {
        return err;
    }

    if (!pollStatus(kTxValidBit, false)) {
        err = ESP_ERR_INVALID_RESPONSE;
        return err;
    }

    err = i2c_.write(device_address_, kWriteReg, &dataToWrite, 1);

    return err;
}


esp_err_t AS726X::virtualReadRegister(uint8_t virtualRegAddr, uint8_t& dataRead) {
    esp_err_t err = ESP_OK;

    if (!pollStatus(kTxValidBit, false)) {
        err = ESP_ERR_INVALID_RESPONSE;
        return err;
    } 

    err = i2c_.write(device_address_, kWriteReg, &virtualRegAddr, 1);
    if (err != ESP_OK) {
        return err;
    }

    if (!pollStatus(kRxValidBit, true)) {
        err = ESP_ERR_INVALID_RESPONSE;
        return err;
    }

    err = i2c_.read(device_address_, kReadReg, &dataRead, 1);

    return err;
}


// esp_err_t AS726X::init(Gain gain, uint8_t measurementMode) {

// }


uint8_t AS726X::getHardwareVersion(void) {
    uint8_t version = 0xFF;
    virtualReadRegister(kAS726xHWVersion, version);
    return version;
}


uint8_t AS726X::getDeviceType(void) {
    uint8_t type = 0xFF;
    virtualReadRegister(kAS726xDeviceType, type);
    return type;
}


uint8_t AS726X::getTemperature(void) {
    uint8_t temperature = 0xFF;
    virtualReadRegister(kAS726xDeviceTemp, temperature);
    return temperature;
}


void AS726X::setMeasurementMode(uint8_t mode) {
    virtualWriteRegister(kAS726xControlSetup, mode);
}


void AS726X::softReset(void) {
    uint8_t value = 0;
    virtualReadRegister(kAS726xControlSetup, value);
    const uint8_t kSoftResetBit = 1 << 7;
    value |= kSoftResetBit;
    virtualWriteRegister(kAS726xControlSetup, value);
}


void AS726X::enableInterrupt(void) {
    uint8_t value = 0;
    virtualReadRegister(kAS726xControlSetup, value);
    const uint8_t kEnableInterruptBit = 1 << 6;
    value |= kEnableInterruptBit;
    virtualWriteRegister(kAS726xControlSetup, value);
}


void AS726X::disableInterrupt(void) {
    uint8_t value = 0;
    virtualReadRegister(kAS726xControlSetup, value);
    const uint8_t kEnableInterruptBit = 1 << 6;
    value &= ~kEnableInterruptBit;
    virtualWriteRegister(kAS726xControlSetup, value);
}


void AS726X::setGain(Gain gain) {
    uint8_t value = 0;
    virtualReadRegister(kAS726xControlSetup, value);
    const uint8_t kEnableInterruptBit = 1 << 6;
    value &= ~kEnableInterruptBit;
    virtualWriteRegister(kAS726xControlSetup, value);
}