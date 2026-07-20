/*******************************************************************************
 *******************************************************************************
 * @file ADXL345.hpp
 * @brief Contains the declarations of the ADXL345 class methods.
 *
 * This sensor has 3 axis accelerometers and can generate up to 2 interrupts by
 * different pins.
 * 
 * @version v0.2.0
 * @date 2025-03-13
 * @author emmanuel@sense-ai.co, Sense AI
 * @author mateor@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include "smart_sensor_sense.hpp"

typedef void (*adxl345Callback_t)(void*);

typedef enum {
    kHigh = 0,
    kLow  = 1
} ActiveLevel;

/**
 * @class ADXL345
 * @brief A class to interface with the ADXL345 accelerometer sensor.
 */
class ADXL345 : public Sensor {
public:
    /**
     * @enum Interrupt
     * @brief Enumeration for the available INTs of the ADXL345
     */
    enum class Interrupt : uint8_t {
        INT1 = 0,
        INT2 = 1
    };

    /**
     * @enum PowerMode
     * @brief Enumeration for Bandwith power modes
     */
    enum class PowerMode : uint8_t {
        kNormal = 0,
        kLow    = 1
    };

    /**
     * @enum OutputDataRate
     * @brief Enumeration for output data rates.
     */
    enum class OutputDataRate : uint8_t {
        k3200Hz = 0b1111,
        k1600Hz = 0b1110,
        k800Hz  = 0b1101,
        k400Hz  = 0b1100,   // Low power {
        k200Hz  = 0b1011,   
        k100Hz  = 0b1010,
        k50Hz   = 0b1001,
        k25Hz   = 0b1000,
        k12_5Hz = 0b0111,   // } Low power
        k6_25Hz = 0b0110,
    };

    /**
     * @enum ActivityControl
     * @brief Enumeration for activity control settings.
     */
    enum class ActivityControl : uint8_t {
        kNone       = 0x00,
        kInactZ     = 0x01,
        kInactY     = 0x02,
        kInactX     = 0x04,
        kInactAcDc  = 0x08,
        kActZ       = 0x10,
        kActY       = 0x20,
        kActX       = 0x40,
        kActAcDc    = 0x80
    };

    /* ActivityControl bitwise operators */
    friend ActivityControl operator | (ActivityControl a, ActivityControl b);
    friend ActivityControl operator & (ActivityControl a, ActivityControl b);
    friend ActivityControl operator ~ (ActivityControl a);

    /**
     * @enum PowerControlBits
     * @brief Enumeration for power control bits.
     */
    enum class PowerControlBits : uint8_t {
        kNone       = 0x00,
        kWakeUp8Hz  = 0x00, // TODO
        kWakeUp4Hz  = 0x01,
        kWakeUp2Hz  = 0x02,
        kWakeUp1Hz  = 0x03,
        kSleep      = 0x04,
        kMeasure    = 0x08,
        kAutoSleep  = 0x10,
        kLink       = 0x20,
    };

    /* PowerControlBits bitwise operators */
    friend PowerControlBits operator | (PowerControlBits a, PowerControlBits b);
    friend PowerControlBits operator & (PowerControlBits a, PowerControlBits b);
    friend PowerControlBits operator ~ (PowerControlBits a);

    /**
     * @enum InterruptFlags
     * @brief Enumeration for interrupt flags.
     */
    enum class InterruptFlags : uint8_t { 
        kNone       = 0x00,
        kOverrun    = 0x01,
        kWatermark  = 0x02,
        kFreeFall   = 0x04,
        kInactivity = 0x08,
        kActivity   = 0x10,
        kDoubleTap  = 0x20,
        kSingleTap  = 0x40,
        kDataReady  = 0x80
    };

    /* InterruptFlags bitwise operators */
    friend InterruptFlags operator | (InterruptFlags a, InterruptFlags b);
    friend InterruptFlags operator & (InterruptFlags a, InterruptFlags b);

    /**
     * @enum FifoMode
     * @brief Enumeration for available FIFO modes
     * @note bypass mode is not included, use resetFIFO() to set this state
     */
    enum class FifoMode : uint8_t {
        kFIFO    = 0x01,
        kStream  = 0x02,
        kTrigger = 0x03
    };

    /**
     * @enum AccelerationRange
     * @brief Enumeration for interrupt flags.
     */
    enum class AccelerationRange : uint8_t {
        k2g  = 0x00,
        k4g  = 0x01,
        k8g  = 0x02,
        k16g = 0x03
    };

    /**
     * @brief Module ID.
     */
    static constexpr uint8_t kDeviceId = 0xE5;

    /**
     * @brief Slave addresses.
     */
    static constexpr uint8_t kAddressAltLow  = 0x53;
    static constexpr uint8_t kAddressAltHigh = 0x1D;

    /**
     * @brief Constructor for ADXL345.
     * @param i2cInstance Reference to the I2C instance.
     * @param address I2C address of the device.
     */
    ADXL345(I2C& i2cInstance, uint8_t address = kAddressAltLow);

    /**
     * @brief Destructor for ADXL345.
     */
    ~ADXL345();

    /**
     * @brief Initializes the sensor. For measuring use enableMeasure().
     * @return esp_err_t Error status.
     */
    esp_err_t init(void) override;

    /**
     * @brief Measures the sensor data.
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
     * @brief Reads the device ID.
     * @return uint8_t Device ID.
     */
    uint8_t readDeviceID(void) const;

    /**
     * @brief Configures activity control.
     * @param config Activity control configuration.
     * @return esp_err_t Error status.
     */
    esp_err_t configureActivityControl(ActivityControl config) const;

    /**
     * @brief Configures activity control threshold.
     * @param threshold The value of the threshold (62.5mg/LSB)
     * @return esp_err_t Error status.
     * @note A value of 0 here can cause unexpected behaviour if the ACT INT
     *       is enabled!.
     */
    esp_err_t configureActivityThreshold(uint8_t threshold) const;

    /**
     * @brief Configures inactivity control threshold.
     * @param threshold The value of the threshold (62.5mg/LSB)
     * @return esp_err_t Error status.
     * @note A value of 0 here can cause unexpected behaviour if the INACT INT
     *       is enabled!.
     */
    esp_err_t configureInactivityThreshold(uint8_t threshold) const;

    /**
     * @brief Configures inactivity time.
     * @param inactivityTime The value of wait time in seconds.
     * @return esp_err_t Error status.
     * @note A value of 0 here can cause unexpected behaviour if the INACT INT
     *       is enabled!.
     */
    esp_err_t configureInactivityTime(uint8_t time) const;

    /**
     * @brief Configures power control.
     * @param config Power control configuration.
     * @return esp_err_t Error status.
     */
    esp_err_t configurePowerControl(PowerControlBits config) const;

    /**
     * @brief Clears power control bits.
     * @param bits Power control bits to clear.
     * @return esp_err_t Error status.
     */
    esp_err_t clearPowerControlBits(PowerControlBits bits) const;

    /**
     * @brief Configures output data rate.
     * @param rate Output data rate.
     * @return esp_err_t Error status.
     */
    esp_err_t configureOutPutDataRate(OutputDataRate rate) const;

    /**
     * @brief Configures output data rate.
     * @param rate Output data rate.
     * @param mode Sensor power mode.
     * @return esp_err_t Error status.
     */
    esp_err_t configureOutPutDataRate(OutputDataRate rate, PowerMode mode) const;

    /**
     * @brief Sets full resolution mode.
     * @return esp_err_t Error status.
     */
    esp_err_t setFullResolution(void);

    /**
     * @brief Configures g range for measurement (sensor's sensibility)
     * @param range Available ranges: +-2g, +-4g, +-8g,+-16g.
     * @return esp_err_t Error status.
     */
    esp_err_t configureAccelerationRange(AccelerationRange range);

    /**
     * @brief Configures FIFO buffer.
     * @param mode Set one of three possible modes.
     * @param samples From 0 to 32 samples are allowed. This will manage the 
     *                watermark Interrupt.
     * @param intNum Set INT1 or INT2 to handle trigger interrupts.
     * @return esp_err_t Error status.
     */
    esp_err_t configureFIFO(FifoMode mode, uint8_t samples, Interrupt intNum) const;

    /**
     * @brief Enables the sensor to perform measures.
     */
    esp_err_t enableMeasure(void) const;

    /**
     * @brief Disables the sensor to perform measures.
     */
    esp_err_t disableMeasure(void) const;

    /**
     * @brief Checks if data from the accelerometers is ready to be read.
     * @return bool Data ready status.
     */
    bool checkDataReady(void) const;

    /**
     * @brief Returns how many samples are in FIFO
     * @return uint8_t current number of samples.
     */
    uint8_t checkFifoStatus(void) const;

    /**
     * @brief Calibrate the sensor internal offsets.
     * @note The sensor must remain quiet for a good calibration.
     */
    esp_err_t calibrate(void);

    /**
     * @brief Returns the last raw data readings from the accelerometer.
     */
    void getRawData(int16_t* buffer) const;

    /**
     * @brief Configures the (INT1 or INT2) for specific interrupts.
     * @param flags Interrupt flags to assign.
     * @param pin Interrupt pin (INT1 or INT2).
     * @return esp_err_t Error status.
     */
    esp_err_t assignInterrupts(InterruptFlags flags, Interrupt intNum) const;

    /**
     * @brief Enables specific interrupts.
     * @param flags Interrupt flags to enable.
     * @return esp_err_t Error status.
     */
    esp_err_t enableInterrupts(InterruptFlags flags);

    /**
     * @brief Disables specific interrupts.
     * @param flags Interrupt flags to disable.
     * @return esp_err_t Error status.
     */
    esp_err_t disableInterrupts(InterruptFlags flags);

    /**
     * @brief Configures GPIO pin connected to INT1 or INT2 for interrupts.
     * @param pin GPIO pin number.
     * @param intNum Which ADXL345 interrupt pin (INT1 or INT2) to associate.
     * @param level If the INT is active high or active low.
     * @param mcuIntEnable Flag to select if the pin should rise interrupts.
     * @return esp_err_t Error status.
     * 
     * @note If Active high is selected, the internal resistor will be pull-down
     * if active low is selected, the internal resistor will be pull-up
     */
    esp_err_t configurePin(gpio_num_t pin, Interrupt intNum, ActiveLevel level,
                           bool mcuIntEnable = true);

    /**
     * @brief Attaches a callback function to the specified interrupt pin.
     *
     * This function registers the given callback function and its associated
     * context for the specified interrupt (INT1 or INT2). It also initializes the
     * shared ISR used for handling interrupts.
     *
     * @param intNum   The interrupt number (INT1 or INT2) to attach the callback.
     * @param callback The callback function to be invoked when the interrupt occurs.
     * @param context  A pointer to user-defined data that will be passed to the
     *                 callback.
     * @return esp_err_t ESP_OK on success, or an error code (e.g.,
     *         ESP_ERR_INVALID_ARG) if the callback, context, or interrupt type is
     *         invalid, or if ISR initialization fails.
     */
    esp_err_t attachInterrupt(Interrupt intNum, adxl345Callback_t callback,
                              void* context);

    /**
    * @brief Detaches the callback function from the specified interrupt pin.
    *
    * This function clears the callback and its associated context for the specified
    * interrupt (INT1 or INT2).
    *
    * @param intNum The interrupt number (INT1 or INT2) from which to detach the
    *               callback.
    * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG if an invalid
    *         interrupt type is provided.
    */
    esp_err_t detachInterrupt(Interrupt intNum);

    /**
    * @brief Enables the GPIO interrupt for the specified interrupt pin.
    *
    * This function adds the ISR handler for the corresponding GPIO pin (associated
    * with INT1 or INT2) using the pre-registered callback context.
    *
    * @param intNum The interrupt number (INT1 or INT2) to enable.
    * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG if an invalid
    *         interrupt type is provided.
    */
    esp_err_t enablePinInterrupt(Interrupt intNum);

    /**
    * @brief Disables the GPIO interrupt for the specified interrupt pin.
    *
    * This function removes the ISR handler for the corresponding GPIO pin
    * (associated with INT1 or INT2).
    *
    * @param intNum The interrupt number (INT1 or INT2) to disable.
    * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_ARG if an invalid
    *         interrupt type is provided.
    */
    esp_err_t disablePinInterrupt(Interrupt intNum);

    esp_err_t getInterruptReason(InterruptFlags& flags) const;

    bool isIntFlagUp(InterruptFlags flag) const;

private:
    static constexpr uint8_t kDevIdReg       = 0x00;
    static constexpr uint8_t kThreshTapReg   = 0x1D;
    static constexpr uint8_t kOfsXReg        = 0x1E;
    static constexpr uint8_t kOfsYReg        = 0x1F;
    static constexpr uint8_t kOfsZReg        = 0x20;
    static constexpr uint8_t kDurReg         = 0x21;
    static constexpr uint8_t kLatentReg      = 0x22;
    static constexpr uint8_t kWindowReg      = 0x23;
    static constexpr uint8_t kThreshActReg   = 0x24;
    static constexpr uint8_t kThreshInactReg = 0x25;
    static constexpr uint8_t kTimeInactReg   = 0x26;
    static constexpr uint8_t kActInactCtlReg = 0x27;
    static constexpr uint8_t kThreshFfReg    = 0x28;
    static constexpr uint8_t kTimeFfReg      = 0x29;
    static constexpr uint8_t kTapAxesReg     = 0x2A;
    static constexpr uint8_t kActTapStatusReg= 0x2B;
    static constexpr uint8_t kBwRateReg      = 0x2C;
    static constexpr uint8_t kPowerCtlReg    = 0x2D;
    static constexpr uint8_t kIntEnableReg   = 0x2E;
    static constexpr uint8_t kIntMapReg      = 0x2F;
    static constexpr uint8_t kIntSourceReg   = 0x30;
    static constexpr uint8_t kDataFormatReg  = 0x31;
    static constexpr uint8_t kDataX0Reg      = 0x32;
    static constexpr uint8_t kDataX1Reg      = 0x33;
    static constexpr uint8_t kDataY0Reg      = 0x34;
    static constexpr uint8_t kDataY1Reg      = 0x35;
    static constexpr uint8_t kDataZ0Reg      = 0x36;
    static constexpr uint8_t kDataZ1Reg      = 0x37;
    static constexpr uint8_t kFifoCtlReg     = 0x38;
    static constexpr uint8_t kFifoStatusReg  = 0x39;

    static bool isIsrServiceInstalled_;
    
    const uint8_t kSensorID_ = 0x20;

    I2C& i2c_;

    uint8_t slaveAddress_ = 0;

    int16_t accelRaw_[3] = {0};

    gpio_num_t int1Gpio_ = GPIO_NUM_NC;
    gpio_num_t int2Gpio_ = GPIO_NUM_NC;

    adxl345Callback_t int1Callback_ = nullptr;
    void* int1Context_ = nullptr;

    adxl345Callback_t int2Callback_ = nullptr;
    void* int2Context_ = nullptr;

    static esp_err_t initializeSharedISR(void);
    static void IRAM_ATTR handleINT1(void* arg);
    static void IRAM_ATTR handleINT2(void* arg);

    /**
     * @brief Reads a register value.
     * @param regAddr Register address.
     * @param value Pointer to store the value.
     * @return esp_err_t Error status.
     */
    esp_err_t readRegister(uint8_t regAddr, uint8_t* value) const;

    /**
     * @brief Writes a value to a register.
     * @param regAddr Register address.
     * @param value Value to write.
     * @return esp_err_t Error status.
     */
    esp_err_t writeRegister(uint8_t regAddr, uint8_t value) const;

    /**
     * @brief Enables the measure bit.
     * @return esp_err_t Error status.
     */
    esp_err_t enableMeasureBit(void) const;

    /**
     * @brief Disables the measure bit.
     * @return esp_err_t Error status.
     */
    esp_err_t disableMeasureBit(void) const;

    /**
     * @brief Collects samples for calibration.
     * @param avgArray Array to store the averaged samples.
     * @return esp_err_t Error status.
     */
    esp_err_t collectSamples(int16_t* avgArray);

    /**
     * @brief Reads current offsets.
     * @param offsetArray Array to store the offsets.
     * @return esp_err_t Error status.
     */
    esp_err_t readCurrentOffsets(int8_t* offsetArray);

    /**
     * @brief Updates the offsets.
     * @param avgArray Array containing averaged samples.
     * @return esp_err_t Error status.
     */
    esp_err_t updateOffsets(int16_t* avgArray);

    /**
     * @brief Calibrates the sensor offsets.
     * @return esp_err_t Error status. TIMEOUT = 30 tries reached.
     * @note The device should be kept steady for a good calibration
     */
    esp_err_t calibrateOffsets(void);

    /**
     * @brief Configures the active level of interrupts (active high/active low).
     * @param activeLow True for active low, false for active high.
     * @return esp_err_t Error status.
     */
    esp_err_t setInterruptActiveLevel(ActiveLevel level) const;
};
