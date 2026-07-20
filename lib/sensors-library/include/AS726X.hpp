/*******************************************************************************
 * @file AS726X.hpp
 * @brief Contains the declarations of the AS726X class methods.
 *
 * @version v0.1.0
 * @date 2024-05-15
 * @author Sense-AI
 *******************************************************************************/

#pragma once

#include <cstdint>

#include "smart_sensor_sense.hpp"

class AS726X : public Sensor {
public:
    AS726X(I2C& i2c, uint8_t device_address = 0x49);
    ~AS726X();

    enum class Gain {
        GAIN_1X = 0,
        GAIN_3_7X = 1,
        GAIN_16X = 2,
        GAIN_64X = 3
    };

    bool begin(Gain gain = Gain::GAIN_64X, uint8_t measurementMode = 3);
    void takeMeasurements(void);


    uint8_t getDeviceType(void);
    uint8_t getHardwareVersion(void);

    void softReset(void);

    void enableInterrupt(void);
    void disableInterrupt(void);

    void setGain(Gain gain);

    void setMeasurementMode(uint8_t mode);

    uint8_t getTemperature(void); // °C


    void takeMeasurementsWithBulb(void);
    bool dataAvailable(void);
    void enableIndicator(void);
    void disableIndicator(void);
    void setIndicatorCurrent(uint8_t current);
    void enableBulb(void);
    void disableBulb(void);
    void setBulbCurrent(uint8_t current);
    void setIntegrationTime(uint8_t integrationValue);

    int getViolet(void);
    int getBlue(void);
    int getGreen(void);
    int getYellow(void);
    int getOrange(void);
    int getRed(void);

    int getR(void);
    int getS(void);
    int getT(void);
    int getU(void);
    int getV(void);
    int getW(void);

    float getCalibratedViolet(void);
    float getCalibratedBlue(void);
    float getCalibratedGreen(void);
    float getCalibratedYellow(void);
    float getCalibratedOrange(void);
    float getCalibratedRed(void);

    float getCalibratedR(void);
    float getCalibratedS(void);
    float getCalibratedT(void);
    float getCalibratedU(void);
    float getCalibratedV(void);
    float getCalibratedW(void);

private:
    I2C& i2c_;
    uint8_t device_address_;

    const uint8_t kAS726XAddr           = 0x49; // 7-bit unshifted default I2C Address

    const uint8_t kSensorTypeAS7262     = 0x3E;
    const uint8_t kSensorTypeAS7263     = 0x3F;
    
    const uint8_t kStatusReg            = 0x00;
    const uint8_t kWriteReg             = 0x01;
    const uint8_t kReadReg              = 0x02;
    const uint8_t kRxValidBit           = 0x01;
    const uint8_t kTxValidBit           = 0x02;

    // Register addresses
    const uint8_t kAS726xDeviceType     = 0x00;
    const uint8_t kAS726xHWVersion      = 0x01;
    const uint8_t kAS726xControlSetup   = 0x04;
    const uint8_t kAS726xIntT           = 0x05;
    const uint8_t kAS726xDeviceTemp     = 0x06;
    const uint8_t kAS726xLEDControl     = 0x07;

    const uint8_t kAS72XXSlaveStatusReg = 0x00;
    const uint8_t kAS72XXSlaveWriteReg  = 0x01;
    const uint8_t kAS72XXSlaveReadReg   = 0x02;

    // AS7262 Registers
    const uint8_t kAS7262V              = 0x08;
    const uint8_t kAS7262B              = 0x0A;
    const uint8_t kAS7262G              = 0x0C;
    const uint8_t kAS7262Y              = 0x0E;
    const uint8_t kAS7262O              = 0x10;
    const uint8_t kAS7262R              = 0x12;
    const uint8_t kAS7262VCal           = 0x14;
    const uint8_t kAS7262BCal           = 0x18;
    const uint8_t kAS7262GCal           = 0x1C;
    const uint8_t kAS7262YCal           = 0x20;
    const uint8_t kAS7262OCal           = 0x24;
    const uint8_t kAS7262RCal           = 0x28;

    // AS7263 Registers
    const uint8_t kAS7263R              = 0x08;
    const uint8_t kAS7263S              = 0x0A;
    const uint8_t kAS7263T              = 0x0C;
    const uint8_t kAS7263U              = 0x0E;
    const uint8_t kAS7263V              = 0x10;
    const uint8_t kAS7263W              = 0x12;
    const uint8_t kAS7263RCal           = 0x14;
    const uint8_t kAS7263SCal           = 0x18;
    const uint8_t kAS7263TCal           = 0x1C;
    const uint8_t kAS7263UCal           = 0x20;
    const uint8_t kAS7263VCal           = 0x24;
    const uint8_t kAS7263WCal           = 0x28;

    const uint8_t kAS72XXSlaveTXValid   = 0x02;
    const uint8_t kAS72XXSlaveRXValid   = 0x01;

    const uint8_t kPollingDelay = 5;

    int getChannel(uint8_t channelRegister);
    float getCalibratedValue(uint8_t calAddress);
    float convertBytesToFloat(uint32_t myLong);
    void clearDataAvailable();

    bool pollStatus(uint8_t bit_mask, bool expected_value);

    esp_err_t writeRegister(uint8_t reg, uint8_t value);
    esp_err_t readRegister(uint8_t reg, uint8_t& value);

    esp_err_t virtualReadRegister(uint8_t virtualAddr, uint8_t& dataRead);
    esp_err_t virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite);
};