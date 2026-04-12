# EMG8 Bracelet

8-channel surface EMG acquisition bracelet based on the **ESP32-S3-WROOM-1-N8**. Four ADS1015 ADCs sample raw EMG and envelope signals simultaneously using interrupt-driven mixed-rate continuous mode, with binary SD logging and real-time UART CSV output.

## Hardware

| Component | Part | Quantity |
|-----------|------|----------|
| MCU | ESP32-S3-WROOM-1-N8 | 1 |
| ADC | ADS1015 (12-bit, 3300 SPS) | 4 |
| IMU | ICM-42605 (6-axis) | 1 |
| Storage | MicroSD (SPI, FATFS) | 1 |
| LED | WS2812 RGB | 1 |
| Switch | Reed (normally open) | 1 |

## Pin Map

### I2C (1 MHz)

| Bus | SDA | SCL | Devices |
|-----|-----|-----|---------|
| I2C0 | GPIO6 | GPIO7 | ADC1 (0x48), ADC2 (0x49) |
| I2C1 | GPIO45 | GPIO47 | ADC3 (0x48), ADC4 (0x49) |

### ADC ALERT/RDY Interrupts

| ADC | GPIO |
|-----|------|
| ADC1 | GPIO40 |
| ADC2 | GPIO41 |
| ADC3 | GPIO42 |
| ADC4 | GPIO15 |

### SD Card (SPI2)

| Signal | GPIO |
|--------|------|
| CS | GPIO10 |
| MOSI | GPIO11 |
| SCK | GPIO12 |
| MISO | GPIO13 |

### IMU — ICM-42605 (SPI3)

| Signal | GPIO |
|--------|------|
| MOSI | GPIO35 |
| SCK | GPIO36 |
| MISO | GPIO37 |
| CS | GPIO38 |
| FSYNC | GPIO39 |
| INT | GPIO16 |

### Peripherals

| Device | GPIO |
|--------|------|
| WS2812 RGB LED | GPIO2 |
| Reed Switch | GPIO9 |

## Channel Layout (per ADC)

| Channel | Function | Rate (All mode) |
|---------|----------|-----------------|
| 0 | Raw EMG | ~1150 Hz |
| 1 | Raw EMG | ~1150 Hz |
| 2 | Envelope | ~57 Hz |
| 3 | Envelope | ~57 Hz |

With 4 ADCs this gives **8 raw EMG channels** and **8 envelope channels**.

## Firmware Architecture

The firmware uses a multi-task design on the ESP32-S3 dual-core processor:

```
Core 0                          Core 1
┌──────────────┐                ┌──────────────┐
│  app_main    │                │  sdWriteTask │  priority 5
│  (main loop) │                │  (batched    │
│              │                │   binary SD) │
├──────────────┤                ├──────────────┤
│  uartTask    │  priority 3   │  ads_mixed   │  priority MAX
│  (~50 Hz CSV)│                │  (×4 ADC     │
└──────────────┘                │   ISR tasks) │
                                └──────────────┘
```

- **ADC tasks** (×4, core 1): ISR-driven via ALERT/RDY pins. Each conversion triggers a semaphore, the task reads the result and calls `onSample()` which enqueues to a FreeRTOS queue.
- **SD writer** (core 1, priority 5): Drains the sample queue in batches of 500 (~4 KB). Files rotate every 60 seconds under a session directory.
- **UART CSV** (core 0, priority 3): Prints latest readings at ~50 Hz with auto-adjusted column headers per mode.
- **Main loop** (core 0): Monitors UART commands and reed switch for mode changes, start/stop, and pause/resume.

## Acquisition Modes

| Command | Mode | Channels | Description |
|---------|------|----------|-------------|
| `1` | All | 0, 1, 2, 3 | Raw EMG (fast) + Envelope (slow, 1/20 divider) |
| `2` | Raw | 0, 1 | Raw EMG only at full speed (~1200 Hz/ch) |
| `3` | Env | 2, 3 | Envelope only at full speed (~1200 Hz/ch) |
| `0` | Stop | — | Stop recording |

Modes can be switched at runtime via UART without rebooting. The reed switch toggles between pause and resume (defaults to All mode on first press).

## LED Status

| Color | Meaning |
|-------|---------|
| Blue | Booting / paused / stopped |
| Green | Recording |
| Red | Init error (ADC or SD failure) |

## SD Binary Format

Each file under `s_<epoch>/` contains:

**Header (16 bytes):**

| Offset | Size | Field |
|--------|------|-------|
| 0 | 4 | Magic `"EMG8"` |
| 4 | 1 | Version (1) |
| 5 | 1 | Number of ADCs (4) |
| 6 | 1 | Channels per ADC (4) |
| 7 | 1 | Slow divider (20) |
| 8 | 4 | File epoch (seconds) |
| 12 | 4 | Reserved |

**Sample records (8 bytes each):**

| Offset | Size | Field |
|--------|------|-------|
| 0 | 4 | Timestamp (µs since recording start) |
| 4 | 1 | ADC index (0–3) |
| 5 | 1 | Channel index (0–3) |
| 6 | 2 | Value (signed 12-bit) |

Files rotate every 60 seconds.

## Building

### Requirements

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/) (6.x+)
- ESP-IDF framework (auto-installed by PlatformIO)

### Compile & Flash

```bash
pio run                    # build
pio run -t upload          # flash
pio device monitor -b 115200   # serial monitor
```

### Library Dependencies

| Library | Branch | Source |
|---------|--------|--------|
| [sensors-library](https://github.com/SenseAI-Organization/sensors-library) | `feature/ads1xxx` | ADS1015 driver with mixed-rate continuous mode |
| [data-logging-library](https://github.com/SenseAI-Organization/data-logging-library) | `dev` | SD card (FATFS), SPI bus management |
| [actuators-library](https://github.com/SenseAI-Organization/actuators-library) | `dev` | RGB LED (WS2812), Switch |

## License

Sense-AI