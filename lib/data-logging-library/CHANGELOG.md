# Changelog

All notable changes to this project will be documented in this file.

<!-- The format is based on [Keep a Changelog](https://keepachangelog.com/), -->
<!-- and this project adheres to [Semantic Versioning](https://semver.org/). -->

## [0.3.5] - 2026-02-04

### Added
- Flash Storage: Added NVS encryption support with HMAC-based key protection
  - New constructor overload with `encrypted` flag and `partitionLabel` parameters
  - Automatic encryption key generation and management in dedicated NVS keys partition
  - Support for HMAC peripheral-based key protection using eFuse storage
  - Conditional compilation support via `CONFIG_NVS_ENCRYPTION` flag
  - When encryption is disabled, simplified constructor without encryption parameters
  - Transparent encryption/decryption for all data operations (put/get/commit)
  - To enable encryption: Configure `CONFIG_NVS_ENCRYPTION=y` in menuconfig
- Flash Storage: Added partition table with encrypted NVS keys partition
  - New `partitions/partitions.csv` with 4KB `nvs_key` partition (encrypted flag)
  - NVS keys partition uses `ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS` subtype
  - Compatible with HMAC-based key protection (CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC)
- Flash Storage: Added automatic handling of corrupted NVS keys partition
  - Detects `ESP_ERR_NVS_CORRUPT_KEY_PART` error during initialization
  - Automatically erases corrupted partition before generating new keys
  - Provides clear error logging for troubleshooting
- Examples: Added `flash_encrypted_example.cpp` demonstrating encrypted storage usage
  - Shows how to initialize encrypted flash storage with custom partition label
  - Demonstrates storing sensitive data (passwords, API keys, device IDs)
  - Examples of multiple encrypted namespaces for different data types
  - Includes comprehensive error handling and status logging

### Changed
- Flash Storage: Constructor signature now conditional based on `CONFIG_NVS_ENCRYPTION`
  - With encryption: `FlashStorage(const char* namespaceName, bool encrypted = false, const char* partitionLabel = "nvs_key")`
  - Without encryption: `FlashStorage(const char* namespaceName)`
  - Private members for encryption (encrypted_, partitionLabel_) only exist when CONFIG_NVS_ENCRYPTION is defined
- Flash Storage: Enhanced `init()` method with encryption initialization path
  - Automatically finds NVS keys partition using `esp_partition_find_first()`
  - Reads or generates encryption configuration based on partition state
  - Falls back to non-encrypted initialization when encryption is disabled or not requested
  - Improved error messages for troubleshooting encryption setup issues
- Flash Storage: Updated include dependencies conditionally
  - `esp_partition.h` only included when `CONFIG_NVS_ENCRYPTION` is enabled
  - Reduces binary size when encryption features are not needed

### Fixed
- Flash Storage: Added handling for `ESP_ERR_NVS_KEYS_NOT_INITIALIZED` error during key reading
- Flash Storage: Improved error handling when NVS keys partition is not found
- Flash Storage: Fixed potential initialization failures with corrupted key partitions

## [0.3.4] - 2026-01-28

### Added
- SD Storage: Added `SD_SENSE_ENABLED` preprocessor directive to enable conditional compilation of SD storage module
  - Header file throws compile-time error if `SD_SENSE_ENABLED` is not defined (unless building internally)
  - Source file implementation wrapped with `#ifdef SD_SENSE_ENABLED` guard
  - Affected files: `sd_storage_sense.hpp`, `sd_storage_sense.cpp`
  - Module must be explicitly enabled by defining `SD_SENSE_ENABLED` in build configuration
  - Prevents accidental usage and provides clear compile-time feedback when not properly configured
  - To enable SD storage: Add `-D SD_SENSE_ENABLED` to build_flags in platformio.ini
  - When disabled: Attempting to include the header will result in a compilation error with instructions

### Changed
- SD Storage: Updated module to require explicit enablement via `SD_SENSE_ENABLED` flag

## [0.3.3] - 2025-09-29

### Added

### Changed
- Updated ESP-IDF framework from version 5.4 to 5.4.1.

### Fixed

## [0.3.2] - 2025-08-21

### Added
- SD Storage: Added `renameObj()` method to rename files and directories on SD card.
- SD Storage: Added `kRename` operation type to `ObjOps` enum for tracking rename operations.

### Changed
- SD Storage: Enhanced object state management to support rename operations.
- SD Storage: Updated `updatePaths()` method to handle rename operation state tracking.

### Fixed
- SD Storage: Improved path management after rename operations.

## [0.3.1] - 2025-07-28

### Added

### Changed
- SD Storage: Replaced compiler warnings with static_assert for configuration validation.
- SD Storage: Improved compile-time validation for CONFIG_FATFS_LFN_HEAP and CONFIG_FATFS_MAX_LFN settings.
- SD Storage: Enhanced error messages with clear instructions for fixing configuration issues.

### Fixed
- SD Storage: Configuration warnings now cause compilation failure.
- SD Storage: Developers can no longer accidentally compile with incorrect FatFs configuration settings.

## [0.2.0] - 2025-06-27

### Added
- SD Storage: Comprehensive documentation for all classes, enums, and methods following Doxygen standards.
- SD Storage: Enhanced error handling with proper resource cleanup in deinit() method.
- SD Storage: Complete SD card filesystem interface with FAT support via SPI communication.
- SD Storage: File and directory management operations (create, delete, navigate, list).
- SD Storage: Advanced file operations including read, write, append, and seek functionality.
- SD Storage: Object attribute management and timestamp setting capabilities.
- SD Storage: Recursive directory search functionality with lookFor() method.
- Flash Storage: Basic flash memory interface for ESP32 SoCs.
- Data Logging: Core data logging functionality for sensor data persistence.

### Changed
- SD Storage: Removed all printf debug statements for cleaner embedded application compatibility.
- SD Storage: Improved memory management with proper resource deallocation.
- SD Storage: Enhanced code structure with better separation of concerns.

### Fixed
- SD Storage: Fixed potential memory leaks in file operations.
- SD Storage: Corrected error handling in mount/unmount operations.
- SD Storage: Resolved resource cleanup issues in destructor and deinit() method.

## [0.1.0] - 2025-04-23

### Added
- Initial release of data-logging-library.
- SD Storage: Basic SD card interface via SPI for ESP32 SoCs.
- SD Storage: FatFs filesystem integration for file operations.
- SD Storage: Support for multiple file opening modes (read-only, read-write, append, overwrite).
- SD Storage: Directory navigation and file management capabilities.
- Flash Storage: Initial flash memory storage interface.
- Data Logging: Basic logging framework for sensor data.
- Integration with SPI general purpose library from sensors-library.

### Changed
-

### Fixed
-
