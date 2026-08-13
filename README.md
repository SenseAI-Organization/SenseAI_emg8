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
│  uartTask    │  priority 3    │  adc0 / adc1 │  priority MAX-2
│  (~50 Hz CSV)│                │  (per-bus    │
└──────────────┘                │   DRDY svc)  │
                                ├──────────────┤
                                │  imuTask     │  priority 4
                                └──────────────┘
```

- **ADC service tasks** (×2, core 1, one per I2C bus): each ALERT/RDY ISR timestamps the conversion and posts its ADC index to the bus's event queue; the task blocks on the queue (no polling), reads the result over the new `i2c_master` driver, and calls `onSample()` which enqueues to a FreeRTOS queue. Running one task per bus lets transactions on the two buses overlap.
- **SD writer** (core 1, priority 5): Drains the sample queues in batches (raw 500 × 8 B ≈ 4 KB). One file set (`R<nnn>.bin`, …) per recording start within the session directory.
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

## UART Interface

The bracelet exposes a command-and-stream interface on `UART0`.

### Serial Settings

| Setting | Value |
|---------|-------|
| Baud rate | `460800` |
| Data bits | `8` |
| Parity | `None` |
| Stop bits | `1` |
| Flow control | `None` |
| Line ending for text commands | `\n` recommended |

The firmware emits a mix of:

- Metadata/status lines prefixed with `#`
- One CSV header line prefixed with `H`
- Repeated CSV data lines prefixed with `D`
- Raw binary payload bytes after a `G<path>` file-transfer command

### Host → Bracelet Commands

| Command | Example | Effect |
|---------|---------|--------|
| `1` | `1` | Start or switch to **All** mode |
| `2` | `2` | Start or switch to **Raw** mode |
| `3` | `3` | Start or switch to **Env** mode |
| `0` | `0` | Stop / pause acquisition |
| `?` | `?` | Query current status |
| `V1` | `V1` | Enable 5V rail |
| `V0` | `V0` | Disable 5V rail |
| `W1` | `W1` | Enable WiFi SoftAP + UDP streaming |
| `W0` | `W0` | Disable WiFi (prints `#NET` stats) |
| `L<id>,<rep>` | `L7,3` | Set current grasp label and repetition |
| `F` | `F` | List files on the SD card |
| `G<path>` | `Gs_AABBCCDDEEFF_1713012345/R000.bin` | Transfer one file as raw binary |

Command notes:

- `L<id>,<rep>` and `G<path>` are line commands. Send a terminating newline, for example `L7,3\n`.
- `1`, `2`, and `3` trigger the firmware countdown before acquisition starts.
- `G<path>` is rejected while recording is active and returns `#ERR:BUSY`.
- `F` and `G` require a mounted SD card. Otherwise the device returns `#ERR:NO_SD`.

### Bracelet → Host Responses

| Prefix | Meaning |
|--------|---------|
| `#READY` | Firmware booted and is ready for commands |
| `#MAC:<hex>` | Device MAC used in session directory names |
| `#INIT:ADC=...,SD=...,IMU=...` | Peripheral init summary |
| `#MODE:<n>` | Current acquisition mode |
| `#CD:<n>` | Countdown tick before recording starts |
| `#CD:ABORT` | Countdown cancelled by sending `0` |
| `#REC` | Recording started or resumed |
| `#PAUSE` | Recording paused |
| `#STOP` | Recording stopped |
| `#CNT:<adc>,<c0>,<c1>,<c2>,<c3>` | Conversions delivered per channel of ADC `<adc>` during the recording that just stopped (4 lines, one per ADC). Fast channels of one ADC should match within ±1; use this to verify channel-rate symmetry. |
| `#LABEL:<id>,<rep>` | Label accepted |
| `#5V:0` / `#5V:1` | 5V rail state |
| `#WIFI:0` / `#WIFI:1` | WiFi radio + streaming state |
| `#NET:<ip>:<port>` | UDP client subscribed at this endpoint |
| `#NET:TX=<n>,ERR=<n>,DROP=<n>` | Streaming stats, printed on `W0` |
| `#STATUS:...` | Current status snapshot |
| `#FLIST:<path>` | Start of SD file listing |
| `#F:<name>,<size>` | One file or directory entry |
| `#FEND` | End of SD file listing |
| `#FDATA:<path>,<bytes>` | File transfer header; raw bytes follow immediately |
| `#FDONE` | File transfer complete |
| `#ERR:<reason>` | Command rejected or failed |

### `#STATUS` Format

The current firmware replies to `?` with:

```text
#STATUS:<mode>,<recording>,<sd_ok>,<imu_ok>,<battery_mV>,<battery_pct>,<raw_drops>,<env_drops>,<imu_drops>
```

Field meanings:

| Field | Meaning |
|-------|---------|
| `mode` | `0=Idle`, `1=All`, `2=Raw`, `3=Env` |
| `recording` | `0` stopped/paused, `1` recording |
| `sd_ok` | `1` if SD storage is available |
| `imu_ok` | `1` if the IMU initialized correctly |
| `battery_mV` | Battery voltage in millivolts |
| `battery_pct` | Battery estimate in percent |
| `raw_drops` | Number of dropped raw EMG samples in the current run |
| `env_drops` | Number of dropped envelope samples in the current run |
| `imu_drops` | Number of dropped IMU samples in the current run |

### CSV Stream Format

When recording is active, the bracelet prints:

- One `H,...` header line at start of recording and again whenever the mode changes.
- Repeated `D,...` data lines at about 50 Hz.

Example header in **All** mode:

```text
H,ts_us,adc1_0,adc1_1,adc1_2,adc1_3,adc2_0,adc2_1,adc2_2,adc2_3,adc3_0,adc3_1,adc3_2,adc3_3,adc4_0,adc4_1,adc4_2,adc4_3,ax,ay,az,gx,gy,gz,label,rep
```

Example data line:

```text
D,123456,81,12,204,198,79,9,201,197,84,11,206,199,82,10,203,196,-0.031,0.004,0.998,0.2,-0.1,0.0,7,3
```

Notes for the Python datalogger:

- `D` lines are low-rate snapshots for monitoring, not the full EMG dataset.
- The high-rate dataset lives on the SD card in `R<nnn>.bin`, `E<nnn>.bin`, `I<nnn>.bin`, and `M<nnn>.bin`, where `<nnn>` is a zero-padded index that increments on every recording start within a session (so pause/resume never overwrites earlier data).
- After `#FDATA:<path>,<bytes>`, read exactly `<bytes>` raw bytes before parsing the trailing `#FDONE` line.
- During file transfer, treat the UART stream as binary, not line-oriented text.

## WiFi / UDP Streaming

Off by default (radio adds 120–250 mA draw). Send `W1` over UART to enable, `W0` to disable.

- The bracelet hosts a WPA2 SoftAP: SSID `EMG8-<MAC>`, password `emg8sense`, bracelet IP `192.168.4.1`.
- Subscribe by sending **any** UDP datagram to `192.168.4.1:3333`; the firmware streams to the sender's address/port from then on. Re-send periodically if your viewer's port may change.
- The stream carries **everything at full rate** (raw + envelope + IMU, ~85 KB/s in All mode). The SD card remains the ground-truth record.

**Packet format** (little-endian, ≤1404 bytes):

| Offset | Size | Field |
|--------|------|-------|
| 0 | 2 | Magic `"E8"` |
| 2 | 1 | Version (1) |
| 3 | 1 | Type: 0 = raw `Sample[]`, 1 = envelope `Sample[]`, 2 = `ImuSample[]` |
| 4 | 4 | Per-type sequence number (gaps ⇒ lost packets) |
| 8 | 2 | Record count |
| 10 | 2 | Reserved |
| 12 | … | Records (8-byte `Sample` or 20-byte `ImuSample`, same layouts as SD) |

Partial batches flush after 30 ms, so envelope/IMU packets arrive promptly even though raw packets fill first (~53 packets/s in All mode).

Minimal Python receiver:

```python
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b"HI", ("192.168.4.1", 3333))          # subscribe
while True:
    pkt = s.recv(2048)
    magic, ver, ptype, seq, count = struct.unpack_from("<2sBBIH", pkt, 0)
    if magic != b"E8":
        continue
    if ptype in (0, 1):                          # raw / envelope
        for i in range(count):
            ts, adc, ch, val = struct.unpack_from("<IBBh", pkt, 12 + 8 * i)
```

## SD Binary Format

Each recording start within a session directory `s_<MAC>_<epoch>/` produces one file set: `M<nnn>.bin`, `R<nnn>.bin`, `E<nnn>.bin`, `I<nnn>.bin`. Only `M<nnn>.bin` has a header — `R`/`E`/`I` are pure record streams with no framing, so file size alone gives the record count.

**`M<nnn>.bin` — master file: 32-byte header, then a stream of 12-byte label events**

| Offset | Size | Field |
|--------|------|-------|
| 0 | 4 | Magic `"EMG8"` |
| 4 | 1 | Version (4) |
| 5 | 1 | Number of ADCs (4) |
| 6 | 1 | Channels per ADC (4) |
| 7 | 1 | Slow divider (20) |
| 8 | 4 | Epoch (seconds **since boot**, not wall-clock — the device has no RTC) |
| 12 | 2 | Battery voltage (mV) at recording start |
| 14 | 1 | Battery percentage at recording start |
| 15 | 1 | Battery state (see `BatteryManager::State`) |
| 16 | 2 | IMU output data rate (Hz) |
| 18 | 6 | Device MAC address |
| 24 | 1 | Mode (`1`=All, `2`=Raw, `3`=Env) |
| 25 | 7 | Reserved |

Label event (12 bytes, repeated for each `L<id>,<rep>` command received while recording):

| Offset | Size | Field |
|--------|------|-------|
| 0 | 4 | Timestamp (µs since recording start) |
| 4 | 2 | Grasp/movement ID |
| 6 | 2 | Repetition |
| 8 | 4 | Reserved |

**`R<nnn>.bin` / `E<nnn>.bin` — raw EMG / envelope: stream of 8-byte sample records, no header**

| Offset | Size | Field |
|--------|------|-------|
| 0 | 4 | Timestamp (µs since recording start, captured in the ADC's DRDY ISR) |
| 4 | 1 | ADC index (0–3) |
| 5 | 1 | Channel index (0–3) |
| 6 | 2 | Value (signed 12-bit) |

**`I<nnn>.bin` — IMU: stream of 20-byte records, no header**

| Offset | Size | Field |
|--------|------|-------|
| 0 | 4 | Timestamp (µs since recording start) |
| 4 | 2×3 | Accel X/Y/Z, milli-g (int16) |
| 10 | 2×3 | Gyro X/Y/Z, deci-dps (int16) |
| 16 | 2 | Temperature × 100 (int16) |
| 18 | 2 | Reserved |

These layouts are defined in [src/emg8_types.hpp](src/emg8_types.hpp) (`Sample`, `ImuSample`, `LabelEvent`) — the same structs are used for the UDP stream records (see above).

## Building

### Requirements

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/) (6.x+)
- ESP-IDF framework (auto-installed by PlatformIO)

### Compile & Flash

```bash
pio run                    # build
pio run -t upload          # flash
pio device monitor -b 460800   # serial monitor (app UART runs at 460800, not the 115200 boot-log rate)
```

Flashing rewrites the partition table (a custom [partitions.csv](partitions.csv): 3 MB app partition on the 8 MB flash, needed for WiFi/lwIP), which erases NVS — nothing in this firmware currently depends on data stored there.

### Library Dependencies

The libraries below are **vendored directly into [lib/](lib/)** as plain files (not git submodules or PlatformIO registry packages), so the repository is self-contained and cloneable without access to the private Sense AI GitHub organization. Each still carries its own `README.md`/`CHANGELOG.md` documenting its own version history.

| Library | Path | Provides |
|---------|------|----------|
| sensors-library | [lib/sensors-library](lib/sensors-library) | ADS1015 driver (mixed-rate continuous mode), ICM-42605 IMU, I2C/SPI wrappers, misc sensors |
| data-logging-library | [lib/data-logging-library](lib/data-logging-library) | SD card (FATFS) and flash storage |
| actuators-library | [lib/actuators-library](lib/actuators-library) | RGB LED (WS2812), reed switch |
| battery-library | [lib/battery-library](lib/battery-library) | Battery voltage/percentage/charge-state monitoring, 5V rail control |

## License

Sense-AI