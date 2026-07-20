# LSM6DSOX IMU Examples

This folder contains comprehensive examples for using the LSM6DSOX 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) sensor.

## Features

The LSM6DSOX library supports:
- **Dual communication interfaces**: I2C and SPI
- **Flexible data rates**: 12.5Hz to 6.67kHz for both sensors
- **Configurable full-scale ranges**:
  - Accelerometer: ±2g, ±4g, ±8g, ±16g
  - Gyroscope: ±250dps, ±500dps, ±1000dps, ±2000dps
- **FIFO buffer**: Up to 512 samples with multiple modes
- **Dual interrupts**: INT1 and INT2 with programmable routing
- **Low power modes**: Various ODR and power-down options
- **Temperature sensor**: Built-in temperature measurement

## Examples Overview

### 1. LSM6DSOX_example.cpp
**Basic I2C example with angle calculation**

Demonstrates:
- Basic sensor initialization via I2C
- Reading accelerometer and gyroscope data
- Reading temperature
- Calculating roll and pitch angles from accelerometer
- Converting raw data to physical units (mg, dps, °C)

**I2C Address being used:**
- SDO/SA0 → GND (I2C address 0x6A)

---

### 2. LSM6DSOX_Orientation_example.cpp
**Orientation example using built-in methods**

Demonstrates:
- Basic sensor initialization via I2C
- Reading accelerometer and gyroscope data
- Calculating roll and pitch angles using `getRoll()` and `getPitch()` class methods
- Simplified application code

**I2C Address being used:**
- SDO/SA0 → GND (I2C address 0x6A)

---

### 3. LSM6DSOX_DeepSleep_WakeUp_example.cpp
**Ultra Low Power Wake-Up Example**

Demonstrates:
- Configuring Accelerometer in Ultra Low Power Mode (12.5Hz, Low Power Mode enabled)
- Disabling Gyroscope for power saving
- Using "Wake-Up" (Activity) interrupt to detect motion
- Putting ESP32 into Deep Sleep and waking up on interrupt

**Note:**
- Uses `enableAccelLowPowerMode(true)` to disable High-Performance mode.
- Uses Wake-Up interrupt instead of Tilt, as Tilt requires High-Performance mode.

**Hardware connections:**
- SDA → GPIO 5
- SCL → GPIO 4
- INT1 → GPIO 19

---

### 4. LSM6DSOX_SPI_example.cpp
**Basic SPI example**

Demonstrates:
- SPI interface configuration
- Sensor initialization via SPI
- Reading sensor data over SPI

**Hardware connections:**
- MOSI → GPIO 23
- MISO → GPIO 19
- SCLK → GPIO 18
- CS → GPIO 5

**Note:** LSM6DSOX supports SPI Mode 0 and Mode 3

---

### 5. LSM6DSOX_FIFO_example.cpp
**FIFO buffering with watermark interrupt**

Demonstrates:
- FIFO configuration in continuous mode
- Setting watermark threshold
- Using interrupt to signal when FIFO has enough data
- Batched data reading

**Use cases:**
- Reducing CPU wake-ups
- Collecting batches of data
- Efficient data logging

**Hardware connections:**
- SDA → GPIO 5
- SCL → GPIO 4
- INT1 → GPIO 19

---

### 6. LSM6DSOX_DeepSleep_example.cpp
**Power optimization with deep sleep**

Demonstrates:
- Using FIFO to buffer data during deep sleep
- Waking on FIFO watermark interrupt
- Processing batched data after wake-up
- Minimizing power consumption

**Key features:**
- Sensor state persists across deep sleep (RTC data)
- Only wake when FIFO has sufficient data
- Ideal for battery-powered applications

**Power savings:**
- Deep sleep: ~10µA (ESP32)
- Sensor continues collecting at configured ODR
- Wake only when needed

**Hardware connections:**
- SDA → GPIO 5
- SCL → GPIO 4
- INT1 → GPIO 19 (configured for EXT0 wakeup)

---

### 7. LSM6DSOX_LowPower_example.cpp
**Different power modes demonstration**

Demonstrates:
- High performance mode (1.67kHz, both sensors)
- Normal mode (104Hz, both sensors)
- Low power mode (12.5Hz, accel only)
- Ultra low power (both sensors off)
- Accelerometer-only mode

**Power consumption comparison:**
- High performance: ~0.9mA
- Normal mode: ~0.5mA
- Low power: ~90µA
- Ultra low power: ~12µA
- Accel only: ~90µA

**Use cases:**
- Motion tracking: Normal mode
- Orientation sensing: Accel-only mode
- Standby: Ultra low power
- High-speed data acquisition: High performance

---

### 8. LSM6DSOX_Interrupt_example.cpp
**Interrupt configuration and usage**

Demonstrates:
- Configuring multiple interrupt sources
- Using both INT1 and INT2 pins
- Data ready interrupts for each sensor
- Interrupt-driven data acquisition

**Features:**
- INT1: Accelerometer data ready
- INT2: Gyroscope data ready
- No polling required - efficient CPU usage

**Hardware connections:**
- SDA → GPIO 5
- SCL → GPIO 4
- INT1 → GPIO 19
- INT2 → GPIO 20

---

### 9. LSM6DSOX_ActivityTilt_example.cpp
**Motion and activity detection (Interrupt Driven)**

Demonstrates:
- Configuring tilt detection logic
- Routing tilt event to INT1 pin
- Handling GPIO interrupts on ESP32
- Wake-up on motion concept

**Parameters:**
- Activity threshold: 200mg
- Consecutive samples: 3
- Detection based on acceleration magnitude

**Use cases:**
- Motion-triggered recording
- Alarm systems
- Tap detection
- Gesture recognition foundation

**Hardware connections:**
- SDA → GPIO 5
- SCL → GPIO 4
- INT1 → GPIO 19

---

### 10. LSM6DSOX_TiltPolling_example.cpp
**Tilt detection using polling**

Demonstrates:
- Configuring tilt detection logic
- Polling the sensor status register
- No GPIO interrupts required

**Use cases:**
- Simple applications where latency is not critical
- Systems without available interrupt pins

---

## Pin Configuration Reference

### I2C (Default for most examples)
```
SDA  → GPIO 5
SCL  → GPIO 4
INT1 → GPIO 19
INT2 → GPIO 20
```

### I2C Address Selection
- SDO/SA0 → GND: Address = 0x6A (default)
- SDO/SA0 → VDD: Address = 0x6B

### SPI (LSM6DSOX_SPI_example.cpp)
```
MOSI → GPIO 23
MISO → GPIO 19
SCLK → GPIO 18
CS   → GPIO 5
```

## Usage Tips

### Interrupt Configuration (New in v0.8.0)
The library uses a split architecture for advanced interrupts (Wake-up, 6D, Tilt):
1. **Configure Logic**: `configureWakeUp()`, `configure6DOrientation()`, etc. sets thresholds and enables internal detection.
2. **Route to Pin**: `enableWakeUpInterrupt()`, `enable6DInterrupt()`, etc. routes the signal to INT1 or INT2.

This allows you to use features in polling mode (without routing to a pin) or interrupt mode.

### Choosing the Right ODR
- **High-speed motion**: 833Hz - 1.67kHz
- **Normal applications**: 104Hz - 208Hz
- **Low power**: 12.5Hz - 52Hz

### Full-Scale Selection
**Accelerometer:**
- ±2g: High resolution, normal movement
- ±4g: Good balance for most applications
- ±8g: Sports, vibration
- ±16g: Impact detection, extreme motion

**Gyroscope:**
- ±250dps: Slow, precise rotation
- ±500dps: Normal rotation speed
- ±1000dps: Fast rotation
- ±2000dps: Very fast rotation

### FIFO Best Practices
- Set watermark to balance latency and efficiency
- Use continuous mode for streaming data
- Use bypass-to-continuous for event-based capture
- Monitor FIFO overrun flag

### Power Optimization
1. Disable unused sensor (accel or gyro)
2. Use lowest ODR that meets requirements
3. Use FIFO with deep sleep for batched operation
4. Consider interrupt-driven architecture

## Sensitivity Values

### Accelerometer (mg/LSB)
- ±2g: 0.061
- ±4g: 0.122
- ±8g: 0.244
- ±16g: 0.488

### Gyroscope (mdps/LSB)
- ±250dps: 8.75
- ±500dps: 17.50
- ±1000dps: 35.0
- ±2000dps: 70.0

### Temperature
- Sensitivity: 1/256 °C/LSB
- Offset: 25°C

## Common Issues and Solutions

### Sensor Not Detected
- Check I2C/SPI connections
- Verify I2C address (SDO/SA0 pin)
- Check power supply (2.5V - 3.6V)
- Verify pull-up resistors on I2C

### Noisy Data
- Add decoupling capacitors near sensor
- Use lower full-scale range if possible
- Consider digital filtering
- Check for mechanical vibration

### High Power Consumption
- Disable unused sensor
- Reduce ODR
- Use FIFO with deep sleep
- Check that sensor enters power-down properly

### Interrupt Not Working
- Verify GPIO configuration
- Check interrupt polarity setting
- Ensure interrupt is enabled in sensor and on GPIO
- Verify callback is attached before enabling

## Further Reading

- LSM6DSOX Datasheet
- Application Notes:
  - AN5192: LSM6DSOX machine learning core
  - AN5272: LSM6DSOX finite state machine
  - AN5282: LSM6DSOX sensor fusion

## License

Part of the Sense AI sensor library ecosystem.
