# Sensors Library
**Version:** v0.4.5

A comprehensive library containing all the classes related to sensors.

## Overview

- **What it does:** Manage sensor data, interface with various hardware sensors, and provide abstractions for ease of use.
- **Why it exists:** To simplify sensor integration in projects, standardize sensor handling, etc.
- **Who it's for:** Developers working with sensor technologies, embedded systems enthusiasts, etc.

## Features

Current classes:
- AnalogSensor
- VoltageDivider
- Battery
- DigitalSensor
- Switch
- ADXL345

## Installation

In order to use this library, be sure to be registered on the Sense AI organization. After that, create an SSH key in order to be able to import it on your platformio project.

```bash
# Clone the repository
git clone https://github.com/SenseAI-Organization/sensors-library.git

# Navigate to the directory
cd sensors-library

# Import this library on platformio (change #dev for the appropiate branch)
lib_deps = git@github.com:SenseAI-Organization/sensors-library.git#dev

