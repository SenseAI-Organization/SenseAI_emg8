# Changelog

All notable changes to this project will be documented in this file.

<!-- The format is based on [Keep a Changelog](https://keepachangelog.com/), -->
<!-- and this project adheres to [Semantic Versioning](https://semver.org/). -->

## [0.1.2] - 2025-09-30

### Added
- Improved RGB class resource management with proper cleanup in destructor

### Changed
- Updated WS2812 timing parameters in RGB class for better LED strip compatibility (bit0: 3,1,9,0 and bit1: 9,1,3,0)
- Improved RGB::sendLatchSignal() implementation using vTaskDelay for more reliable reset signal timing
- Enhanced RGB::setColor() method to allow color changes regardless of LED state (removed state validation check)
- Updated RGB::cycle() method implementation for proper on/off cycling with delays
- Refined RGB::updateHardware() methods with better error handling and state management

### Fixed
- Fixed RGB class RMT resource cleanup to prevent memory leaks and handle cleanup properly in destructor
- Improved RGB initialization sequence with proper error handling and resource cleanup on failure
- Fixed WS2812 protocol timing issues that could cause incorrect color display or LED strip malfunction
- Enhanced RGB hardware update reliability with better handle validation and error reporting

## [0.1.1] - 2025-09-29

### Added
- 

### Changed
- Updated ESP-IDF framework from version 5.4 to 5.4.1.

### Fixed
- 

## [0.1.0] - 2024-08-15

### Added
- Initial release of actuators-library.
- Actuator: Base abstract class interface for all actuator types.
- Actuator: Virtual methods for init, turnOn, turnOff, pulse, and cycle operations.
- LED: Complete LED actuator implementation with GPIO control.
- LED: Support for basic on/off operations and state management.
- LED: Pulse functionality with configurable duration.
- LED: Cycle functionality for repetitive operations.
- RGB: Advanced RGB LED actuator with RMT-based control.
- RGB: Full color control with 8-bit RGB intensity values.
- RGB: Hardware-accelerated color output using ESP32 RMT peripheral.
- RGB: Custom RMT encoder for precise timing control.
- RGB: Color mixing and intensity management.
- RGB: State tracking and color retrieval functionality.
- Examples: Simple LED control example demonstrating basic usage.
- Examples: RGB LED example showing color manipulation and effects.
- Integration with ESP32 GPIO and RMT drivers for hardware control.

### Changed
- 

### Fixed
-