# Actuators Library

![Version](https://img.shields.io/badge/version-0.1.2-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32-orange.svg)
![Framework](https://img.shields.io/badge/framework-ESP--IDF-blueviolet.svg)

A comprehensive library containing all the classes related to actuator control, light management, and output device interfaces for the Sense AI ecosystem.

## Table of Contents

- [Project Overview](#project-overview)
- [Key Features](#key-features)
- [Library Architecture](#library-architecture)
- [Modules Documentation](#modules-documentation)
  - [Actuator Interface](#actuator-interface)
  - [LED](#led)
  - [RGB](#rgb)
- [Installation](#installation)
- [Contribution Guidelines](#contribution-guidelines)
- [Project Contacts](#project-contacts)

## Project Overview

The `actuators-library` provides standardized interfaces for controlling output devices in ESP32-based projects. It simplifies hardware abstraction for simple binary actuators and complex addressable ones.

- **What it does:** Manages LEDs, RGB strips (WS2812), and generic actuators with a unified API.
- **Why it exists:** To standardise the way the application interacts with hardware outputs, abstracting GPIO and RMT details.
- **Who it's for:** Developers working with hardware indicators, lighting feedback, and signal outputs within the Sense AI ecosystem.

## Key Features

- **Unified Interface**: Abstract `Actuator` base class for polymorphic control.
- **GPIO Management**: Robust `LED` class for standard binary outputs.
- **RGB Integration**: Hardware-accelerated (RMT) control for WS2812 (NeoPixel) LEDs.
- **Timing Utilities**: Built-in methods for `pulse` and `cycle` visualization patterns.

## Library Architecture

The library is structured around a base interface and specific implementations:

### Core Modules

- **`Actuator`**: Abstract base class defining the contract (`turnOn`, `turnOff`, `pulse`, `cycle`).
- **`LED`**: Implementation for standard GPIO-connected single-color LEDs.
- **`RGB`**: Implementation for WS2812B addressable LEDs using the ESP32 RMT peripheral.

## Modules Documentation

### Actuator Interface

**Purpose**: Defines the standard behavior for any output device.

**Key Definition**:
```cpp
virtual esp_err_t init(void) = 0;
virtual esp_err_t turnOn(void) = 0;
virtual esp_err_t turnOff(void) = 0;
virtual esp_err_t pulse(uint32_t millisOn) = 0;
virtual esp_err_t cycle(uint32_t millisOn) = 0;
```

### LED

**Purpose**: Manages basic LED actuators using GPIO pins.

**Key Features**:
- Simple on/off logic.
- Blocking `pulse` implementation using FreeRTOS delays.
- Configures pins as outputs automatically.

### RGB

**Purpose**: Manages RGB LED actuators using WS2812 protocol with the RMT peripheral.

**Key Features**:
- **RMT Driven**: Uses the Remote Control peripheral for precise timing without CPU blocking during transmission.
- **Color Mgmt**: Set colors via `setColor(r, g, b)`.
- **Safety**: Handles hardware initialization and cleanup.

## Installation

In order to use this library, be sure to be registered on the Sense AI organization. After that, create an SSH key in order to be able to import it on your platformio project.

```bash
# Clone the repository
git clone https://github.com/SenseAI-Organization/actuators-library.git

# Navigate to the directory
cd actuators-library

# Import this library on platformio (change #dev for the appropiate branch)
lib_deps = git@github.com:SenseAI-Organization/actuators-library.git#dev
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
- **Contributors**: Emmanuel (emmanuel@sense-ai.co)
- **Development Team**: Sense AI Organization
