# Data Logging Library

![Version](https://img.shields.io/badge/version-0.3.5-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32-orange.svg)
![Framework](https://img.shields.io/badge/framework-ESP--IDF-blueviolet.svg)

A comprehensive library containing all the classes related to data logging, debugging, and storage management for the Sense AI ecosystem.

## Table of Contents

- [Project Overview](#project-overview)
- [Key Features](#key-features)
- [Library Architecture](#library-architecture)
- [Modules Documentation](#modules-documentation)
  - [Debugger](#debugger)
  - [Flash Storage](#flash-storage)
  - [SD Storage](#sd-storage)
- [Installation](#installation)
- [Contribution Guidelines](#contribution-guidelines)
- [Project Contacts](#project-contacts)

## Project Overview

The `data-logging-library` provides essential tools for data persistence and application debugging in ESP32-based projects. It simplifies the interface with hardware storage media and offers flexible logging utilities.

- **What it does:** Manages data logging, provides debugging utilities, and interfaces with storage media like Flash memory (NVS) and SD cards.
- **Why it exists:** To simplify data persistence, standardize debugging output, and provide robust file management abstractions for embedded projects.
- **Who it's for:** Developers working with data logging systems, embedded storage, and debugging tools within the Sense AI ecosystem.

## Key Features

- **Conditional Debugging**: Lightweight debugger class with global enable/disable flags.
- **Flash Memory Management**: Easy wrapper for ESP32 Non-Volatile Storage (NVS) with optional hardware-accelerated encryption for sensitive data.
- **SD Card Integration**: comprehensive interface for SD card operations using SPI and FatFs.
- **Data Persistence**: Robust methods for reading and writing data across restart cycles.

## Library Architecture

The library is structured into modules covering different storage and logging needs:

### Core Modules

- **`debugger_sense`**: Conditional serial output helper.
- **`flash_sense`**: Wrapper for ESP32 NVS (Non-Volatile Storage).
- **`sd_storage_sense`**: SD card management via SPI.

## Modules Documentation

### Debugger

**Purpose**: A utility class for conditional debugging output. It allows developers to add debug prints throughout the code that can be globally enabled or disabled.

**Key Features**:
- Enable/Disable flag in constructor.
- Overloaded `print` methods for various data types (char, int, string, etc.).

**Basic Usage**:
```cpp
#include "data_logging_sense.hpp"

// Initialize debugger with true (active)
Debugger debug(true);

void loop() {
    debug.print("System running...");
}
```

### Flash Storage

**Purpose**: Manages non-volatile storage (NVS) on ESP32 devices, allowing for persistent storage of small data items like configurations or state variables.

**Key Features**:
- Namespace support for organizing data.
- Read/Write operations for various primitive types (uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, strings, and binary blobs).
- Error handling with `esp_err_t`.
- **NVS Encryption**: Optional hardware-accelerated encryption for sensitive data using HMAC-based key protection.
- **Automatic Key Management**: Automatic generation and secure storage of encryption keys in dedicated NVS partition.
- **Transparent Encryption**: Seamless encryption/decryption during all read/write operations.

**Basic Usage**:
```cpp
#include "flash_sense.hpp"

// Non-encrypted storage
FlashStorage storage("config");
storage.init();

// Store and retrieve data
storage.put("wifi_ssid", "MyNetwork");
storage.put("sensor_interval", (uint32_t)5000);
storage.commit();

std::string ssid;
storage.get("wifi_ssid", ssid);
```

**Encrypted Storage Usage**:
```cpp
#include "flash_sense.hpp"

// Encrypted storage (requires CONFIG_NVS_ENCRYPTION=y)
FlashStorage secureStorage("secrets", true);  // Second parameter enables encryption
secureStorage.init();

// Store sensitive data - automatically encrypted
secureStorage.put("api_key", "your_secret_key");
secureStorage.put("password", "user_password");
secureStorage.commit();

// Data is transparently decrypted when retrieved
std::string apiKey;
secureStorage.get("api_key", apiKey);
```

**Configuration**:

To use NVS encryption features, you need to enable encryption support in ESP-IDF configuration:

#### Enabling NVS Encryption via Menuconfig (Recommended)

1. **Open menuconfig**:
   ```bash
   pio run -t menuconfig
   ```

2. **Enable NVS Encryption**:
   - Navigate to: `Component config` → `NVS`
   - Enable `Enable NVS encryption` (CONFIG_NVS_ENCRYPTION)
   - Under `NVS Security Provider`, select `Using HMAC peripheral` (CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC)
   - Set `eFuse key ID` - **Important**: Verify the key ID is within your device's valid range:
     - **ESP32-S3**: eFuse Key ID range: 0-5
     - **ESP32**: Check your specific variant's datasheet
   - Save and exit (press `S`, then `Q`)

3. **Add Custom Partition Table** to your `platformio.ini`:
   ```ini
   board_build.partitions = partitions/partitions.csv
   ```

4. **Create partition table** at `partitions/partitions.csv`:
   ```csv
   # Name,   Type, SubType, Offset,  Size, Flags
   nvs_key,  data, nvs_keys, 0x9000, 0x1000, encrypted
   nvs,      data, nvs,     0xA000, 0x6000,
   phy_init, data, phy,     0x10000, 0x1000,
   factory,  app,  factory, 0x20000, 2M,
   ```
   or just add the nvs_key partition and fix the offsets to use your current partition table.

**Encryption Features**:
- **HMAC-Based Protection**: Encryption keys are protected using ESP32's HMAC peripheral with eFuse-stored keys
- **Automatic Key Generation**: First initialization automatically generates and stores encryption keys
- **Partition Recovery**: Automatic handling of corrupted key partitions with regeneration capability
- **Multiple Namespaces**: Support for multiple encrypted namespaces with different security contexts
- **Backward Compatible**: When encryption is disabled, uses simplified constructor without encryption parameters

**Constructor Signatures**:
- **With Encryption Enabled** (CONFIG_NVS_ENCRYPTION=y):
  ```cpp
  FlashStorage(const char* namespaceName, bool encrypted = false, const char* partitionLabel = "nvs_key")
  ```
- **Without Encryption** (CONFIG_NVS_ENCRYPTION not defined):
  ```cpp
  FlashStorage(const char* namespaceName)
  ```

**Security Considerations**:
- Encryption keys are stored in a dedicated NVS partition (`nvs_key`) marked as encrypted
- HMAC-based protection prevents extraction of encryption keys even with physical access
- Each namespace can have independent encryption settings
- Ideal for storing: API credentials, passwords, private keys, sensitive device configurations

> **Note**: NVS encryption requires a custom partition table with an `nvs_key` partition. When using menuconfig, the NVS encryption settings are stored in the `sdkconfig` file. See the `examples/flash_encrypted_example.cpp` for a complete implementation example.

### SD Storage

**Purpose**: A class to manage SD card storage operations via SPI interface.

**Key Features**:
- **SPI Interface**: Uses standard SPI for communication.
- **FatFs Support**: Full file system support including long filenames.
- **File Management**: Create, read, write, and delete files.
- **Directory Operations**: Navigate, list, and search through directories.
- **Conditional Compilation**: Can be disabled to reduce binary size when not needed.

**Configuration**:

There are two ways to configure SD storage functionality:

#### Option 1: Using Build Flags (Quick Setup)

Add the following build flags to your `platformio.ini`:

```ini
build_flags =
    -D SD_SENSE_ENABLED=1
    -D CONFIG_FATFS_LFN_HEAP=1
    -D CONFIG_FATFS_MAX_LFN=255
```

#### Option 2: Using Menuconfig (Recommended)

For more control over FatFS configuration, use ESP-IDF's menuconfig system:

1. **Open menuconfig**:
   ```bash
   pio run -t menuconfig
   ```

2. **Configure FatFS settings**:
   - Navigate to: `Component config` → `FAT Filesystem support`
   - Set `Long filename support` to `Long filename buffer allocated on heap`
   - Set `Max long filename length` to `255`
   - Save and exit (press `S`, then `Q`)

3. **Add the SD_SENSE_ENABLED flag** to your `platformio.ini`:
   ```ini
   build_flags =
       -D SD_SENSE_ENABLED=1
   ```

**Flag Descriptions**:
- `SD_SENSE_ENABLED`: Enables SD storage module in the data-logging library (required - must be defined)
- `CONFIG_FATFS_LFN_HEAP=1`: Enables long filename support in FatFs, allocated on heap (required)
- `CONFIG_FATFS_MAX_LFN=255`: Sets maximum long filename length to 255 characters (required)

> **Note**: The SD storage module requires `SD_SENSE_ENABLED` to be defined. If not defined, compilation will fail with an error message. Simply omit including the SD storage header files in your project if you don't need SD card functionality. When using menuconfig, the FatFS settings are stored in the `sdkconfig` file and don't need to be specified as build flags.

## Installation

In order to use this library, be sure to be registered on the Sense AI organization. After that, create an SSH key in order to be able to import it on your platformio project.

```bash
# Clone the repository
git clone https://github.com/SenseAI-Organization/data-logging-library.git

# Navigate to the directory
cd data-logging-library

# Import this library on platformio (change #dev for the appropiate branch)
lib_deps = git@github.com:SenseAI-Organization/data-logging-library.git#dev
```

## Contribution Guidelines

To maintain code quality and consistency, please adhere to the following guidelines:

### Branching Strategy
- **Features**: `feature/<module-name>-<feature-description>`
- **Bug Fixes**: `bugfix/<issue-number>-<short-description>`
- **Hotfixes**: `hotfix/<critical-issue>`

### Code Standards
- **Documentation**: Doxygen-style comments for all public APIs.
- **Style Guide**: Follow [Sense AI C++ Style Guide](https://docs.google.com/document/d/1WN5O45172jIPNVFQegHM0BYzg2NH3YsH/edit).
- **Examples**: Include example code in the `examples/` directory.

## Project Contacts

- **Library Maintainer**: Mateo R.B. (mateor@sense-ai.co)
- **Development Team**: Sense AI Organization
