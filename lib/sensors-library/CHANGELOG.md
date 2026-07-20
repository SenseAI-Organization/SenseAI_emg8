# Changelog

All notable changes to this project will be documented in this file.

<!-- The format is based on [Keep a Changelog](https://keepachangelog.com/), -->
<!-- and this project adheres to [Semantic Versioning](https://semver.org/). -->

## [0.8.3] - 2026-01-27
### Added
- **UART**: Added `rxInternalPullup` parameter to base constructor to enable internal pull-up resistor on RX pin during initialization.
- **UART**: Added `setRxPinInternalPullup(bool enable)` method to enable/disable internal pull-up resistor on RX pin at runtime.
- **UART**: Added `getRxPinInternalPullup()` getter method to retrieve current RX pin internal pull-up state.
- **UART**: Added `allowEvents` parameter to all simplified constructors for consistent event handler control across all constructor variants.

### Changed
- **UART**: Updated `init()` method to configure GPIO pull-up mode using `gpio_set_pull_mode()` for consistent pull resistor handling.
- **UART**: Refactored pull-up configuration to use `GPIO_PULLUP_ONLY` and `GPIO_FLOATING` modes for clearer intent.
- **Examples**: Updated `uart_basic_example.cpp` to demonstrate RX internal pull-up configuration using the new setter method.

### Fixed
- **UART**: Added missing `driver/gpio.h` include in implementation file for GPIO pull-up/pull-down functions.

## [0.8.2] - 2025-12-02
### Added
- **LSM6DSOX**: Added `LSM6DSOX_DeepSleep_WakeUp_example.cpp` to demonstrate deep sleep wake-up functionality.
- **LSM6DSOX**: Added `LSM6DSOX_ReedSwitch_DataLogger.cpp` and `LSM6DSOX_Gyro_ReedSwitch_DataLogger.cpp` for data logging scenarios.
- **LSM6DSOX**: Added `AccelPowerMode` and `GyroPowerMode` enums for precise power mode configuration.
- **LSM6DSOX**: Added `configureAccelPowerMode()` and `configureGyroPowerMode()` methods.
- **LSM6DSOX**: Added `enableAccelLowPowerMode()` (deprecated) for backward compatibility.
- **Reed Switch**: Added `examples/reed_switch_example.cpp` to demonstrate basic reed switch usage.

### Fixed
- **LSM6DSOX**: Corrected `setInterruptLatching` to use the correct bit (Bit 7) for LIR.
- **LSM6DSOX**: Fixed `configureWakeUp` to correctly enable interrupts in `TAP_CFG2` register instead of `TAP_CFG0`.

## [0.8.1] - 2025-11-26
### Added
- **LSM6DSOX**: Added `LSM6DSOX_Orientation_example.cpp` to demonstrate simplified roll/pitch calculation using built-in class methods.

### Changed
- **LSM6DSOX**: Improved `getRoll()` algorithm to use a robust formula (`atan2(Ay, sqrt(Ax^2 + Az^2))`). This prevents inaccuracies and gimbal lock when the device is tilted (non-zero pitch), matching the robustness of `getPitch()`.
- **LSM6DSOX**: Updated README to include the new orientation example.

## [0.8.0] - 2025-11-25
### Added
- **LSM6DSOX**: Added the new class and examples
- **LSM6DSOX**: New `WakeUpSrcFlags` and `AllIntSrcFlags` enum classes for robust interrupt source checking.
- **LSM6DSOX**: New `getWakeUpSrc()` and `getAllIntSrc()` methods to retrieve detailed interrupt status.
- **LSM6DSOX**: New `SixDThreshold` enum class for type-safe 6D orientation configuration.
- **LSM6DSOX**: New `enableWakeUpInterrupt()`, `enable6DInterrupt()`, and `enableTiltInterrupt()` methods to explicitly route internal events to physical interrupt pins.

### Changed
- **LSM6DSOX**: Refactored interrupt architecture to separate sensor configuration (logic) from interrupt routing (pin enabling).
- **LSM6DSOX**: Updated `configure6DOrientation()` to use `SixDThreshold` enum and removed implicit interrupt routing.
- **LSM6DSOX**: Updated `checkWakeUp()` to use the new `WakeUpSrcFlags` for more detailed status reporting.
- **LSM6DSOX**: Fixed bitmasking logic in `configureWakeUp()` and `configure6DOrientation()` to prevent overwriting other register bits (Read-Modify-Write).
- **LSM6DSOX**: Corrected `initializeSharedISR()` to use default flags (0) for GPIO ISR service installation.

### Fixed
- **LSM6DSOX**: Resolved an issue where `configureWakeUp` was incorrectly modifying the `MD1_CFG` register without user intent.
- **LSM6DSOX**: Fixed magic numbers in 6D orientation threshold configuration.

## [0.7.0] - 2025-09-26
### Added
-
### Changed
- Espressif32 version, from v5.4.0 to v5.4.1.
- Corrected turbine_flow_meter file name to turbine_flow_meter_example.cpp.

### Fixed
- A missing import in the ADXL345_WaveDetection_example.

## [0.6.4] - 2025-09-12
### Added
-
### Changed
-

### Fixed
- **AnalogSensor**: Resolved a bug with AnalogSensor destructor that was causing a temporal object to crash the device.

## [0.6.2] - 2025-08-05
### Added
-
### Changed
- **AnalogSensor**: Enabled simultaneous use of sensors across both ADC1 and ADC2 peripherals by completely refactoring the underlying resource management.
- **AnalogSensor**: Constructors now automatically detect the correct ADC unit from a GPIO pin, simplifying sensor configuration and preventing errors.

### Fixed
- **AnalogSensor**: Resolved a core architectural flaw that previously prevented using more than one ADC unit in an application.
- **AnalogSensor**: Corrected a bug where the .configure() method would incorrectly hardcode the peripheral to ADC1, overriding the correct auto-detected unit.

## [0.6.1] - 2025-08-01
### Added
- **AnalogSensor**: Robust ADC calibration support using `adc_cali_create_scheme_curve_fitting` for significantly more accurate voltage readings on supported chips.
- **AnalogSensor**: New methods `enableCalibration()` and `isCalibrationEnabled()` to give users explicit control over the calibration feature.
- **Battery**: New flexible constructors to allow specifying custom voltage divider resistor values, removing the hardcoded 1:1 ratio.
- **Docs**: Detailed Doxygen documentation for `getCalibratedMv()` and `readCalibrated()` methods.

### Changed
- **AnalogSensor**: Refactored to use static shared resources (`s_adcUnitHandle_`, `s_adcCaliHandle_`) for the ADC unit, improving efficiency and resource management when using multiple analog sensors on the same ADC peripheral.
- **VoltageDivider & Battery**: Now inherit measurement logic from the base `AnalogSensor::measure()` method, reducing code duplication and centralizing the ADC read process.
- **Code Quality**: Replaced all C-style casts with modern C++ casts (`static_cast`, `reinterpret_cast`) for improved type safety and code clarity.
- **Examples**: Updated `battery_example.cpp` and `voltage_divider_example.cpp` to demonstrate new features, use `ESP_LOG` for better output, and follow best practices.
- **Project**: Increased `clang-format` line length from 80 to 90 characters for better readability.
- **Project**: Updated `.gitignore` to exclude `.vscode/settings.json`.

### Fixed
- **AnalogSensor**: `measure()` no longer fails if ADC calibration is not supported. It now gracefully falls back to using raw ADC values, making sensor readings dramatically more robust across different ESP32 chips.
- **VoltageDivider**: Corrected a potential integer overflow in the input voltage calculation by using a `uint64_t` for intermediate steps.


## [0.6.0] - 2025-06-24
### Added
- TurbineFlowMeter: new class to manage flowmeter sensors that work with digital pulses.
- DYP_A22YYMW: New class to manage this ultrasonic reference sensors.
### Changed
-

### Fixed
-

## [0.6.0] - 2025-06-27

### Added
- SPI: Comprehensive documentation for all classes, enums, and methods following Doxygen standards.
- SPI: Enhanced deinit() method with proper error handling and return values.
- SPI: Mode-specific deinitialization for both master and slave modes.
- SPI: Detailed clock mode documentation with CPOL/CPHA explanations.

### Changed
- SPI: Updated deinit() method signature from void to esp_err_t for better error reporting.
- SPI: Improved resource cleanup with proper slave interface deinitialization.
- SPI: Enhanced documentation with comprehensive parameter descriptions and usage notes.

### Fixed
- SPI: Fixed incomplete resource cleanup in deinit() method for slave mode.
- SPI: Corrected deinitialization sequence to prevent resource leaks.

## [0.5.0] - 2025-03-13
### Added
- UART: new UART class to manage this peripheral see uart_eventHandler_example.cpp

### Changed
-

### Fixed
-

### Removed

- ADXL345: Function checkInterruptSource (Not in use).

## [0.4.6] - 2025-03-13
### Added
- ADXL345: FIFO usage methods and examples.
- ADXL345: MCU Sleep examples.
- ADXL345: Configuration options for the low power mode and its baudrates.
- ADXL345: A method to set the full resolution of the sensor.
- ADXL345: A method to configure the sensibility of the sensor (G range).

### Changed
- ADXL345: Refactored some variables that didn't match the guide of coding and style for the InterruptsFlag enum class.
- ADXL345: Refactored some variables to make the names a little shorter:
 - ActiveLevel::kActiveLow -> ActiveLevel::kLow
 - ActiveLevel::kActiveHigh -> ActiveLevel::kHigh
 - PowerMode::kLowPowerMode -> PowerMode::kLow
 - PowerMode::kNormalPowerMode -> PowerMode::kNormal


### Fixed

## [0.4.x] - 2025-03-25
### Added
- ADXL345: Autosleep mode example.
- ADXL345: Function configureInactivityTime defined.

## [0.4.3] - 2025-02-27
### Added
-

### Changed
-

### Fixed
- An error with the header of the configurePin() method.
- An issue with the date format on the doxygen documentation.


## [0.4.2] - 2025-02-27
### Added
- A new argument for the refactored method configurePin() that lets the user select if the pin should generate interrupts or just change the state of the pin but without interrupt generation.

### Changed
- Refactored configurePinInterrupt() to configurePin().

### Fixed
-
