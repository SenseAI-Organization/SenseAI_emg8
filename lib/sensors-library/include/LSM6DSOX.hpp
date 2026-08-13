/*******************************************************************************
 *******************************************************************************
 * @file LSM6DSOX.hpp
 * @brief Contains the declarations of the LSM6DSOX class methods.
 *
 * This sensor has 3-axis accelerometer and 3-axis gyroscope with advanced
 * features including FIFO, interrupts, and embedded functions.
 *
 * @version v0.1.0
 * @date 2025-11-18
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "esp_attr.h"
#include "smart_sensor_sense.hpp"
#include "spi_gp_sense.hpp"

typedef void (*lsm6dsoxCallback_t)(void*);

/**
 * @enum CommInterface
 * @brief Enumeration for communication interface type
 */
enum class CommInterface : uint8_t { kI2C = 0, kSPI = 1 };

/**
 * @class LSM6DSOX
 * @brief A class to interface with the LSM6DSOX 6-axis IMU sensor.
 */
class LSM6DSOX : public Sensor {
public:
    /**
     * @enum Interrupt
     * @brief Enumeration for the available INTs of the LSM6DSOX
     */
    enum class Interrupt : uint8_t { kInt1 = 0, kInt2 = 1 };

    /**
     * @enum SixDThreshold
     * @brief Thresholds for 6D orientation detection.
     */
    enum class SixDThreshold : uint8_t {
        k50Degrees = 0x03,  // 11: 50 degrees
        k60Degrees = 0x02,  // 10: 60 degrees
        k70Degrees = 0x01,  // 01: 70 degrees
        k80Degrees = 0x00   // 00: 80 degrees
    };

    /**
     * @enum AccelOutputDataRate
     * @brief Enumeration for accelerometer output data rates.
     */
    enum class AccelOutputDataRate : uint8_t {
        kPowerDown = 0x00,
        k12_5Hz = 0x01,
        k26Hz = 0x02,
        k52Hz = 0x03,
        k104Hz = 0x04,
        k208Hz = 0x05,
        k417Hz = 0x06,
        k833Hz = 0x07,
        k1667Hz = 0x08,
        k3333Hz = 0x09,
        k6667Hz = 0x0A
    };

    /**
     * @enum GyroOutputDataRate
     * @brief Enumeration for gyroscope output data rates.
     */
    enum class GyroOutputDataRate : uint8_t {
        kPowerDown = 0x00,
        k12_5Hz = 0x01,
        k26Hz = 0x02,
        k52Hz = 0x03,
        k104Hz = 0x04,
        k208Hz = 0x05,
        k417Hz = 0x06,
        k833Hz = 0x07,
        k1667Hz = 0x08,
        k3333Hz = 0x09,
        k6667Hz = 0x0A
    };

    /**
     * @enum AccelFullScale
     * @brief Enumeration for accelerometer full-scale selection.
     */
    enum class AccelFullScale : uint8_t {
        k2g = 0x00,
        k16g = 0x01,
        k4g = 0x02,
        k8g = 0x03
    };

    /**
     * @enum GyroFullScale
     * @brief Enumeration for gyroscope full-scale selection.
     */
    enum class GyroFullScale : uint8_t {
        k250dps = 0x00,
        k500dps = 0x01,
        k1000dps = 0x02,
        k2000dps = 0x03
    };

    /**
     * @enum CalibrationMode
     * @brief Enumeration for calibration modes.
     */
    enum class CalibrationMode {
        kSmart,  // Preserves gravity (1g) on the active axis
        kTare    // Zeros all axes (removes gravity)
    };

    /**
     * @enum BatchDataRate
     * @brief Enumeration for FIFO batch data rates (BDR).
     */
    enum class BatchDataRate : uint8_t {
        kNotBatched = 0x00,
        k12_5Hz = 0x01,
        k26Hz = 0x02,
        k52Hz = 0x03,
        k104Hz = 0x04,
        k208Hz = 0x05,
        k417Hz = 0x06,
        k833Hz = 0x07,
        k1667Hz = 0x08,
        k3333Hz = 0x09,
        k6667Hz = 0x0A
    };

    /**
     * @enum FifoMode
     * @brief Enumeration for FIFO modes.
     */
    enum class FifoMode : uint8_t {
        kBypass = 0x00,
        kFifo = 0x01,
        kContinuous = 0x06,
        kContinuousToFIFO = 0x03,
        kBypassToContinuous = 0x04
    };

    /**
     * @enum FifoTag
     * @brief FIFO data tag identifiers (from FIFO_DATA_OUT_TAG register)
     */
    enum class FifoTag : uint8_t {
        kGyroNC = 0x01,         // Gyroscope data (not compressed)
        kAccelNC = 0x02,        // Accelerometer data (not compressed)
        kTemperature = 0x03,    // Temperature data
        kTimestamp = 0x04,      // Timestamp data
        kCfgChange = 0x05,      // Configuration change
        kAccelNC_T_2 = 0x06,    // Accel data at T+2
        kAccelNC_T_1 = 0x07,    // Accel data at T+1
        kAccel_2xC = 0x08,      // Accel data 2x compressed
        kAccel_3xC = 0x09,      // Accel data 3x compressed
        kGyro_NC_T_2 = 0x0A,    // Gyro data at T+2
        kGyro_NC_T_1 = 0x0B,    // Gyro data at T+1
        kGyro_2xC = 0x0C,       // Gyro data 2x compressed
        kGyro_3xC = 0x0D,       // Gyro data 3x compressed
        kSensorHub = 0x0E,      // Sensor hub data
        kStepCounter = 0x12,    // Step counter data
        kSensorHub_Nack = 0x19  // Sensor hub NACK
    };

    /**
     * @struct FifoData
     * @brief Structure representing one FIFO word (tag + 6 bytes of data)
     */
    struct FifoData {
        FifoTag tag;
        uint8_t data[6];
    };

    /**
     * @enum InterruptFlags
     * @brief Enumeration for interrupt flags (INT1_CTRL / INT2_CTRL).
     * Note: Some flags like Boot or Temp are specific to INT1 or INT2.
     */
    enum class InterruptFlags : uint8_t {
        kNone = 0x00,
        kAccelDataReady = 0x01,  // INTx_DRDY_XL
        kGyroDataReady = 0x02,   // INTx_DRDY_G
        kBootStatus = 0x04,      // INT1_BOOT (INT1 only) / INT2_DRDY_TEMP (INT2 only)
        kFifoThreshold = 0x08,   // INTx_FIFO_TH
        kFifoOverrun = 0x10,     // INTx_FIFO_OVR
        kFifoFull = 0x20,        // INTx_FIFO_FULL
    };

    /* InterruptFlags bitwise operators */
    friend InterruptFlags operator|(InterruptFlags a, InterruptFlags b);
    friend InterruptFlags operator&(InterruptFlags a, InterruptFlags b);
    friend InterruptFlags operator~(InterruptFlags a);

    /**
     * @enum WakeUpSrcFlags
     * @brief Flags for WAKE_UP_SRC register (0x1B).
     */
    enum class WakeUpSrcFlags : uint8_t {
        kNone = 0x00,
        kZWakeUp = 0x01,         // Z_WU
        kYWakeUp = 0x02,         // Y_WU
        kXWakeUp = 0x04,         // X_WU
        kWakeUpStatus = 0x08,    // WU_IA
        kSleepState = 0x10,      // SLEEP_STATE
        kFreeFallStatus = 0x20,  // FF_IA
        kSleepChange = 0x40      // SLEEP_CHANGE_IA
    };

    /* WakeUpSrcFlags bitwise operators */
    friend WakeUpSrcFlags operator|(WakeUpSrcFlags a, WakeUpSrcFlags b);
    friend WakeUpSrcFlags operator&(WakeUpSrcFlags a, WakeUpSrcFlags b);
    friend WakeUpSrcFlags operator~(WakeUpSrcFlags a);

    /**
     * @enum AllIntSrcFlags
     * @brief Flags for ALL_INT_SRC register (0x1A).
     */
    enum class AllIntSrcFlags : uint8_t {
        kNone = 0x00,
        kFreeFall = 0x01,       // FF_IA
        kWakeUp = 0x02,         // WU_IA
        kSingleTap = 0x04,      // SINGLE_TAP_IA
        kDoubleTap = 0x08,      // DOUBLE_TAP_IA
        k6DOrientation = 0x10,  // D6D_IA
        kSleepChange = 0x20,    // SLEEP_CHANGE_IA
        kTimestampEnd = 0x80    // TIMESTAMP_ENDCOUNT
    };

    /* AllIntSrcFlags bitwise operators */
    friend AllIntSrcFlags operator|(AllIntSrcFlags a, AllIntSrcFlags b);
    friend AllIntSrcFlags operator&(AllIntSrcFlags a, AllIntSrcFlags b);
    friend AllIntSrcFlags operator~(AllIntSrcFlags a);

    /**
     * @brief Module WHO_AM_I ID.
     */
    static constexpr uint8_t kDeviceId = 0x6C;

    /**
     * @brief Slave addresses.
     */
    static constexpr uint8_t kAddressLow = 0x6A;   // SDO/SA0 to ground
    static constexpr uint8_t kAddressHigh = 0x6B;  // SDO/SA0 to VDD

    /**
     * @brief Constructor for LSM6DSOX using I2C.
     * @param i2cInstance Reference to the I2C instance.
     * @param address I2C address of the device.
     */
    LSM6DSOX(I2C& i2cInstance, uint8_t address = kAddressLow);

    /**
     * @brief Constructor for LSM6DSOX using SPI.
     * @param spiInstance Reference to the SPI instance.
     * @param deviceHandle Pointer to SPI device handle.
     */
    LSM6DSOX(SPI& spiInstance, spi_device_handle_t* deviceHandle);

    /**
     * @brief Destructor for LSM6DSOX.
     */
    ~LSM6DSOX();

    /**
     * @brief Initializes the sensor.
     * @return esp_err_t Error status.
     */
    esp_err_t init(void) override;

    /**
     * @brief Measures the sensor data (both accelerometer and gyroscope).
     * @return esp_err_t Error status.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Gets the sensor report.
     * @param _buff Buffer to store the report.
     * @return esp_err_t Error status.
     */
    esp_err_t getReport(char* _buff) const override;

    /**
     * @brief Gets the sensor ID.
     * @return uint8_t Sensor ID.
     */
    uint8_t getID(void) const override;

    /**
     * @brief Reads the device WHO_AM_I register.
     * @return uint8_t Device ID.
     */
    uint8_t readDeviceID(void) const;

    /**
     * @brief Configures accelerometer output data rate.
     * @param rate Output data rate.
     * @return esp_err_t Error status.
     */
    esp_err_t configureAccelDataRate(AccelOutputDataRate rate) const;

    /**
     * @brief Configures gyroscope output data rate.
     * @param rate Output data rate.
     * @return esp_err_t Error status.
     */
    esp_err_t configureGyroDataRate(GyroOutputDataRate rate) const;

    /**
     * @brief Configures accelerometer full-scale range.
     * @param scale Full-scale selection.
     * @return esp_err_t Error status.
     */
    esp_err_t configureAccelFullScale(AccelFullScale scale);

    /**
     * @brief Configures gyroscope full-scale range.
     * @param scale Full-scale selection.
     * @return esp_err_t Error status.
     */
    esp_err_t configureGyroFullScale(GyroFullScale scale);

    /**
     * @brief Configures FIFO mode, watermark, and batch data rates.
     * @param mode FIFO mode selection.
     * @param watermark FIFO watermark level (0-511).
     * @param accelRate Accelerometer batch data rate (default: Not Batched).
     * @param gyroRate Gyroscope batch data rate (default: Not Batched).
     * @return esp_err_t Error status.
     */
    esp_err_t configureFIFO(FifoMode mode, uint16_t watermark,
                            BatchDataRate accelRate = BatchDataRate::kNotBatched,
                            BatchDataRate gyroRate = BatchDataRate::kNotBatched) const;

    /**
     * @brief Calibrates the accelerometer offsets.
     *
     * Calculates the average bias and writes to the User Offset registers.
     *
     * @param mode Calibration mode (Smart or Tare).
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t calibrate(CalibrationMode mode = CalibrationMode::kSmart);

    /**
     * @brief Enables accelerometer measurement.
     * @param rate Output data rate (default: 104Hz).
     * @return esp_err_t Error status.
     */
    esp_err_t enableAccel(AccelOutputDataRate rate = AccelOutputDataRate::k104Hz) const;

    /**
     * @brief Enables gyroscope measurement.
     * @param rate Output data rate (default: 104Hz).
     * @return esp_err_t Error status.
     */
    esp_err_t enableGyro(GyroOutputDataRate rate = GyroOutputDataRate::k104Hz) const;

    /**
     * @brief Disables accelerometer measurement.
     * @return esp_err_t Error status.
     */
    esp_err_t disableAccel(void) const;

    /**
     * @brief Disables gyroscope measurement.
     * @return esp_err_t Error status.
     */
    esp_err_t disableGyro(void) const;

    /**
     * @brief Checks if accelerometer data is ready.
     * @return bool Data ready status.
     */
    bool checkAccelDataReady(void) const;

    /**
     * @brief Checks if gyroscope data is ready.
     * @return bool Data ready status.
     */
    bool checkGyroDataReady(void) const;

    /**
     * @brief Reads temperature data.
     * @return raw temperature reading
     */
    int16_t readTemperature(void);

    /**
     * @brief Measures temperature data.
     * @return esp_err_t Error status.
     */
    esp_err_t measureTemperature(void);

    /**
     * @brief Returns the last raw accelerometer data readings.
     * @param buffer Buffer to store the data (must be at least 3 elements).
     */
    void getAccelRawData(int16_t* buffer) const;

    /**
     * @brief Returns the last raw gyroscope data readings.
     * @param buffer Buffer to store the data (must be at least 3 elements).
     */
    void getGyroRawData(int16_t* buffer) const;

    /**
     * @brief Returns the last temperature reading in LSB.
     * @return int16_t Temperature in raw LSB.
     */
    int16_t getTemperatureRaw(void) const;

    /**
     * @brief Returns the last temperature reading in degrees Celsius.
     * @return float Temperature in degrees Celsius.
     */
    float getTemperatureCelsius(void) const;

    /**
     * @brief Enables specific interrupts on INT1 or INT2.
     * @param flags Interrupt flags to enable.
     * @param intNum Interrupt pin (INT1 or INT2).
     * @return esp_err_t Error status.
     */
    esp_err_t enableInterrupts(InterruptFlags flags, Interrupt intNum);

    /**
     * @brief Disables specific interrupts.
     * @param flags Interrupt flags to disable.
     * @param intNum Interrupt pin (INT1 or INT2).
     * @return esp_err_t Error status.
     */
    esp_err_t disableInterrupts(InterruptFlags flags, Interrupt intNum);

    /**
     * @brief Configures GPIO pin connected to INT1 or INT2 for interrupts.
     * @param pin GPIO pin number.
     * @param intNum Which LSM6DSOX interrupt pin (INT1 or INT2) to associate.
     * @param activeHigh If true, interrupt is active high; if false, active low.
     * @param mcuIntEnable Flag to select if the pin should trigger interrupts.
     * @return esp_err_t Error status.
     */
    esp_err_t configurePin(gpio_num_t pin, Interrupt intNum, bool activeHigh = true,
                           bool mcuIntEnable = true);

    /**
     * @brief Attaches a callback function to the specified interrupt pin.
     * @param intNum The interrupt number (INT1 or INT2) to attach the callback.
     * @param callback The callback function to be invoked when the interrupt occurs.
     * @param context A pointer to user-defined data passed to the callback.
     * @return esp_err_t ESP_OK on success, or an error code.
     */
    esp_err_t attachInterrupt(Interrupt intNum, lsm6dsoxCallback_t callback,
                              void* context);

    /**
     * @brief Detaches the callback function from the specified interrupt pin.
     * @param intNum The interrupt number (INT1 or INT2) from which to detach.
     * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG.
     */
    esp_err_t detachInterrupt(Interrupt intNum);

    /**
     * @brief Enables the GPIO interrupt for the specified interrupt pin.
     * @param intNum The interrupt number (INT1 or INT2) to enable.
     * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG.
     */
    esp_err_t enablePinInterrupt(Interrupt intNum);

    /**
     * @brief Disables the GPIO interrupt for the specified interrupt pin.
     * @param intNum The interrupt number (INT1 or INT2) to disable.
     * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG.
     */
    esp_err_t disablePinInterrupt(Interrupt intNum);

    /**
     * @brief Enables or disables interrupt latching.
     * @param enable If true, interrupts are latched until the source is read.
     * @return esp_err_t Error status.
     */
    esp_err_t setInterruptLatching(bool enable);

    /**
     * @brief Configures the Wake-Up detection parameters.
     * @param threshold Wake-up threshold (1 LSb = FS_XL / 64).
     * @param duration Wake-up duration (1 LSb = 1 / ODR_XL).
     * @return esp_err_t Error status.
     */
    esp_err_t configureWakeUp(uint8_t threshold, uint8_t duration);

    /**
     * @brief Enables/Disables the Wake-Up interrupt on the specified pin.
     * @param enable Enable or disable the interrupt.
     * @param pin Interrupt pin (INT1 or INT2).
     * @return esp_err_t Error status.
     */
    esp_err_t enableWakeUpInterrupt(bool enable, Interrupt pin = Interrupt::kInt1);

    /**
     * @brief Checks if a Wake-Up event has occurred.
     * @return bool True if Wake-Up event detected.
     */
    bool checkWakeUp(void) const;

    /**
     * @brief Configures the 6D Orientation detection.
     * @param threshold Threshold in degrees (50, 60, 70, 80). Default 60.
     * @return esp_err_t Error status.
     */
    esp_err_t configure6DOrientation(SixDThreshold threshold = SixDThreshold::k60Degrees);

    /**
     * @brief Enables/Disables the 6D Orientation interrupt on the specified pin.
     * @param enable Enable or disable the interrupt.
     * @param pin Interrupt pin (INT1 or INT2).
     * @return esp_err_t Error status.
     */
    esp_err_t enable6DInterrupt(bool enable, Interrupt pin = Interrupt::kInt1);

    /**
     * @brief Checks if a 6D Orientation change has occurred.
     * @param position Optional pointer to store the current position (0-5).
     *                 0: XL, 1: XH, 2: YL, 3: YH, 4: ZL, 5: ZH.
     * @return bool True if 6D event detected (orientation changed).
     */
    bool check6DOrientation(uint8_t* position = nullptr) const;

    /**
     * @brief Configures the Relative Tilt detection.
     * @return esp_err_t Error status.
     */
    esp_err_t configureTiltDetection(void);

    /**
     * @brief Enables/Disables the Tilt interrupt on the specified pin.
     * @param enable Enable or disable the interrupt.
     * @param pin Interrupt pin (INT1 or INT2).
     * @return esp_err_t Error status.
     */
    esp_err_t enableTiltInterrupt(bool enable, Interrupt pin = Interrupt::kInt1);

    /**
     * @brief Checks if a Tilt event has occurred.
     * @return bool True if Tilt event detected.
     */
    bool checkTilt(void) const;

    /**
     * @enum AccelPowerMode
     * @brief Power modes for the accelerometer.
     */
    enum class AccelPowerMode : uint8_t {
        kHighPerformance = 0x00,  // XL_HM_MODE = 0
        kLowPowerNormal = 0x01,   // XL_HM_MODE = 1
        kUltraLowPower = 0x02     // XL_ULP_EN = 1 (Requires specific hardware support)
    };

    /**
     * @enum GyroPowerMode
     * @brief Power modes for the gyroscope.
     */
    enum class GyroPowerMode : uint8_t {
        kHighPerformance = 0x00,  // G_HM_MODE = 0
        kLowPowerNormal = 0x01    // G_HM_MODE = 1
    };

    /**
     * @brief Configures the accelerometer power mode.
     * @param mode Power mode selection.
     * @return esp_err_t Error status.
     */
    esp_err_t configureAccelPowerMode(AccelPowerMode mode);

    /**
     * @brief Configures the gyroscope power mode.
     * @param mode Power mode selection.
     * @return esp_err_t Error status.
     */
    esp_err_t configureGyroPowerMode(GyroPowerMode mode);

    /**
     * @brief Enables or disables the Accelerometer Low-Power Mode (High-Performance
     * Disable).
     * @deprecated Use configureAccelPowerMode instead.
     *
     * When enabled (true), the High-Performance mode is disabled (XL_HM_MODE = 1).
     * This reduces power consumption but may limit some features (like embedded
     * functions).
     *
     * @param enable True to enable Low-Power Mode, False to enable High-Performance Mode
     * (default).
     * @return esp_err_t Error status.
     */
    esp_err_t enableAccelLowPowerMode(bool enable);

    /**
     * @brief Performs software reset of the device.
     * @return esp_err_t Error status.
     */
    esp_err_t softwareReset(void) const;

    /**
     * @brief Reads FIFO status.
     * @return uint16_t Number of unread words (samples) in FIFO.
     */
    uint16_t getFifoStatus(void) const;

    /**
     * @brief Reads one word from FIFO (1 tag byte + 6 data bytes).
     * @param fifoData Pointer to FifoData structure to store the read data.
     * @return esp_err_t Error status.
     */
    esp_err_t readFifoWord(FifoData* fifoData) const;

    /**
     * @brief Reads multiple words from FIFO.
     * @param fifoData Pointer to array of FifoData structures.
     * @param numWords Number of words to read.
     * @return esp_err_t Error status.
     */
    esp_err_t readFifoWords(FifoData* fifoData, uint16_t numWords) const;

    /**
     * @brief Parses accelerometer data from FIFO word.
     * @param fifoData Pointer to FifoData containing accelerometer data.
     * @param accelData Output array for X, Y, Z accelerometer values (int16_t[3]).
     * @return bool True if tag indicates accelerometer data, false otherwise.
     */
    bool parseFifoAccelData(const FifoData* fifoData, int16_t* accelData) const;

    /**
     * @brief Parses gyroscope data from FIFO word.
     * @param fifoData Pointer to FifoData containing gyroscope data.
     * @param gyroData Output array for X, Y, Z gyroscope values (int16_t[3]).
     * @return bool True if tag indicates gyroscope data, false otherwise.
     */
    bool parseFifoGyroData(const FifoData* fifoData, int16_t* gyroData) const;

    /**
     * @brief Get the pitch angle (rotation around Y-axis) in degrees.
     * @return Pitch angle in degrees.
     */
    float getPitch(void) const;

    /**
     * @brief Get the roll angle (rotation around X-axis) in degrees.
     * @return Roll angle in degrees.
     */
    float getRoll(void) const;

    /**
     * @brief Reads the WAKE_UP_SRC register.
     * @return WakeUpSrcFlags Flags indicating the wake-up source.
     */
    WakeUpSrcFlags getWakeUpSrc(void) const;

    /**
     * @brief Reads the ALL_INT_SRC register.
     * @return AllIntSrcFlags Flags indicating all interrupt sources.
     */
    AllIntSrcFlags getAllIntSrc(void) const;

private:
    // Register addresses
    static constexpr uint8_t kFuncCfgAccessReg = 0x01;
    static constexpr uint8_t kPinCtrlReg = 0x02;
    static constexpr uint8_t kEmbFuncEnAReg = 0x04;
    static constexpr uint8_t kEmbFuncInitAReg = 0x66;
    static constexpr uint8_t kPageRwReg = 0x17;
    static constexpr uint8_t kEmbFuncInt1Reg = 0x0A;
    static constexpr uint8_t kEmbFuncStatusReg = 0x12;
    static constexpr uint8_t kEmbFuncStatusMainpageReg = 0x35;
    static constexpr uint8_t kFifoCtrl1Reg = 0x07;
    static constexpr uint8_t kFifoCtrl2Reg = 0x08;
    static constexpr uint8_t kFifoCtrl3Reg = 0x09;
    static constexpr uint8_t kFifoCtrl4Reg = 0x0A;
    static constexpr uint8_t kInt1CtrlReg = 0x0D;
    static constexpr uint8_t kInt2CtrlReg = 0x0E;
    static constexpr uint8_t kWhoAmIReg = 0x0F;
    static constexpr uint8_t kCtrl1XlReg = 0x10;
    static constexpr uint8_t kCtrl2GReg = 0x11;
    static constexpr uint8_t kCtrl3CReg = 0x12;
    static constexpr uint8_t kCtrl4CReg = 0x13;
    static constexpr uint8_t kCtrl5CReg = 0x14;
    static constexpr uint8_t kCtrl6CReg = 0x15;
    static constexpr uint8_t kCtrl7GReg = 0x16;
    static constexpr uint8_t kCtrl8XlReg = 0x17;
    static constexpr uint8_t kCtrl9XlReg = 0x18;
    static constexpr uint8_t kCtrl10CReg = 0x19;
    static constexpr uint8_t kAllIntSrcReg = 0x1A;
    static constexpr uint8_t kWakeUpSrcReg = 0x1B;
    static constexpr uint8_t kTapSrcReg = 0x1C;
    static constexpr uint8_t kD6dSrcReg = 0x1D;
    static constexpr uint8_t kStatusReg = 0x1E;
    static constexpr uint8_t kOutTempLReg = 0x20;
    static constexpr uint8_t kOutTempHReg = 0x21;
    static constexpr uint8_t kOutXLGReg = 0x22;
    static constexpr uint8_t kOutXHGReg = 0x23;
    static constexpr uint8_t kOutYLGReg = 0x24;
    static constexpr uint8_t kOutYHGReg = 0x25;
    static constexpr uint8_t kOutZLGReg = 0x26;
    static constexpr uint8_t kOutZHGReg = 0x27;
    static constexpr uint8_t kOutXLXlReg = 0x28;
    static constexpr uint8_t kOutXHXlReg = 0x29;
    static constexpr uint8_t kOutYLXlReg = 0x2A;
    static constexpr uint8_t kOutYHXlReg = 0x2B;
    static constexpr uint8_t kOutZLXlReg = 0x2C;
    static constexpr uint8_t kOutZHXlReg = 0x2D;
    static constexpr uint8_t kFifoStatus1Reg = 0x3A;
    static constexpr uint8_t kFifoStatus2Reg = 0x3B;
    static constexpr uint8_t kFifoDataOutTagReg = 0x78;
    static constexpr uint8_t kFifoDataOutXLReg = 0x79;
    static constexpr uint8_t kFifoDataOutXHReg = 0x7A;
    static constexpr uint8_t kFifoDataOutYLReg = 0x7B;
    static constexpr uint8_t kFifoDataOutYHReg = 0x7C;
    static constexpr uint8_t kFifoDataOutZLReg = 0x7D;
    static constexpr uint8_t kFifoDataOutZHReg = 0x7E;
    static constexpr uint8_t kTapCfg0Reg = 0x56;
    static constexpr uint8_t kTapCfg1Reg = 0x57;
    static constexpr uint8_t kTapCfg2Reg = 0x58;
    static constexpr uint8_t kTapThs6dReg = 0x59;
    static constexpr uint8_t kWakeUpThsReg = 0x5B;
    static constexpr uint8_t kWakeUpDurReg = 0x5C;
    static constexpr uint8_t kMd1CfgReg = 0x5E;
    static constexpr uint8_t kMd2CfgReg = 0x5F;
    static constexpr uint8_t kXOfsUsrReg = 0x73;
    static constexpr uint8_t kYOfsUsrReg = 0x74;
    static constexpr uint8_t kZOfsUsrReg = 0x75;

    static bool isIsrServiceInstalled_;

    const uint8_t kSensorID_ = 0x23;

    CommInterface commInterface_;

    // I2C members
    I2C* i2c_ = nullptr;
    uint8_t slaveAddress_ = 0;

    // SPI members
    SPI* spi_ = nullptr;
    spi_device_handle_t* spiDeviceHandle_ = nullptr;
    int16_t accelRaw_[3] = {0};
    int16_t gyroRaw_[3] = {0};
    int16_t tempRaw_ = 0;

    AccelFullScale accelScale_ = AccelFullScale::k2g;
    GyroFullScale gyroScale_ = GyroFullScale::k250dps;

    gpio_num_t int1Gpio_ = GPIO_NUM_NC;
    gpio_num_t int2Gpio_ = GPIO_NUM_NC;

    lsm6dsoxCallback_t int1Callback_ = nullptr;
    void* int1Context_ = nullptr;

    lsm6dsoxCallback_t int2Callback_ = nullptr;
    void* int2Context_ = nullptr;

    static esp_err_t initializeSharedISR(void);
    static void IRAM_ATTR handleINT1(void* arg);
    static void IRAM_ATTR handleINT2(void* arg);

    /**
     * @brief Reads a register value.
     * @param regAddr Register address.
     * @param value Pointer to store the value.
     * @param len Number of bytes to read.
     * @return esp_err_t Error status.
     */
    esp_err_t readRegister(uint8_t regAddr, uint8_t* value, size_t len = 1) const;

    /**
     * @brief Writes a value to a register.
     * @param regAddr Register address.
     * @param value Value to write.
     * @param len Number of bytes to write.
     * @return esp_err_t Error status.
     */
    esp_err_t writeRegister(uint8_t regAddr, uint8_t value) const;

    /**
     * @brief Reads multiple registers via I2C.
     */
    esp_err_t readRegisterI2C(uint8_t regAddr, uint8_t* data, size_t len) const;

    /**
     * @brief Writes to a register via I2C.
     */
    esp_err_t writeRegisterI2C(uint8_t regAddr, uint8_t value) const;

    /**
     * @brief Reads multiple registers via SPI.
     */
    esp_err_t readRegisterSPI(uint8_t regAddr, uint8_t* data, size_t len) const;

    /**
     * @brief Writes to a register via SPI.
     */
    esp_err_t writeRegisterSPI(uint8_t regAddr, uint8_t value) const;
};
