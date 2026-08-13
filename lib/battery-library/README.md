# BatteryManager Library

A comprehensive battery management library for ESP32 that provides accurate voltage and charge percentage monitoring with charging state detection and intelligent data stabilization.

## Features

- **Accurate Voltage Measurement**: Uses ADC with calibration support via voltage divider
- **Charge Percentage Calculation**: Linear mapping based on configurable min/max voltage
- **Charging Detection**: Monitors charging pin to detect active charging state
- **Power Connection Detection**: Monitors Pgood pin to detect connection to mains power
- **Data Stabilization**: Moving average filter to smooth noisy readings
- **State-Aware Operation**: Different behaviors based on charging/connected/disconnected states
- **Flexible Configuration**: Easy-to-use API with sensible defaults

## Hardware Requirements

1. **Battery Voltage Divider**: Two resistors to scale battery voltage to ADC range (0-3.3V)
   - Recommended: 10kΩ + 10kΩ for 1:1 division (for batteries up to 3.3V input)
   - For higher voltages, adjust resistor ratio accordingly

2. **Charging Detection Pin**: Digital GPIO connected to charging indicator
   - HIGH = Charging active
   - LOW = Not charging

3. **Power Good (Pgood) Pin**: Digital GPIO connected to power connection indicator
   - HIGH = Connected to mains power
   - LOW = Disconnected (running on battery)

## Data Analysis & Findings

Based on extensive real-world testing with your ESP32-S3 device:

### Observed Behavior

| State | Voltage Range | Percentage Range | Characteristics |
|-------|--------------|------------------|-----------------|
| **Charging** | 4156-4182 mV | 95-98% | High, stable readings |
| **Connected (Full)** | 4028-4164 mV | 82-96% | Fluctuating readings |
| **Disconnected** | 3790-3804 mV | 59-60% | Stable, accurate readings |

### Key Insights

1. **Voltage Fluctuation When Connected**: When connected to power (even if not actively charging), voltage readings fluctuate significantly (±13%). This is due to:
   - Charging circuitry switching
   - Power supply noise
   - Battery cell balancing

2. **Stable Readings When Disconnected**: Battery readings are most accurate when running purely on battery power.

3. **Moving Average Necessity**: A 5-sample moving average filter effectively stabilizes readings without introducing excessive lag.

4. **State-Aware Interpretation**: The library reports both raw and filtered values, allowing applications to make informed decisions based on connection state.

## Installation

1. Copy `BatteryManager.hpp` to your project's `include/` folder
2. Copy `BatteryManager.cpp` to your project's `src/` folder
3. Ensure you have the `voltage_divider_sense` library (included in sensors-library)

## Basic Usage

```cpp
#include "BatteryManager.hpp"

// Define your GPIO pins
#define BATTERY_ADC_PIN  GPIO_NUM_8
#define CHARGING_PIN     GPIO_NUM_9
#define PGOOD_PIN        GPIO_NUM_10

// Create BatteryManager instance (simple constructor)
BatteryManager battery(BATTERY_ADC_PIN, CHARGING_PIN, PGOOD_PIN);

// Initialize
battery.init();

// In your main loop
while (true) {
    // Perform measurement
    battery.measure();
    
    // Get readings
    uint16_t voltage = battery.getVoltage();      // Filtered voltage
    uint8_t percent = battery.getPercentage();    // Filtered percentage
    bool charging = battery.isCharging();
    bool connected = battery.isConnected();
    
    printf("Battery: %u%%, %u mV, %s\n", 
           percent, voltage, battery.getStateString());
    
    vTaskDelay(pdMS_TO_TICKS(2000));
}
```

## Advanced Configuration

```cpp
#include "BatteryManager.hpp"

// Create custom configuration struct
battery_config_t config = {
    .battery_adc_pin = GPIO_NUM_8,
    .charging_pin = GPIO_NUM_9,
    .pgood_pin = GPIO_NUM_10,
    .r1 = 10000,      // Upper resistor (Ω)
    .r2 = 10000,      // Lower resistor (Ω)
    .min_voltage = 2800,
    .max_voltage = 4062,
    .filter_samples = 5,
    .attenuation = ADC_ATTEN_DB_12,
    .resolution = ADC_BITWIDTH_DEFAULT
};

// Customize voltage range for your battery type
config.min_voltage = 3200;  // Empty voltage (mV) = 0%
config.max_voltage = 4200;  // Full voltage (mV) = 100%

// Adjust filtering (1-20 samples)
config.filter_samples = 5;  // More samples = smoother but slower response

// ADC settings
config.attenuation = ADC_ATTEN_DB_12;  // 0-3.3V range
config.resolution = ADC_BITWIDTH_DEFAULT;  // 12-bit

// Create BatteryManager with custom config
BatteryManager battery(config);
battery.init();
```

## API Reference

### Initialization

```cpp
esp_err_t init(void);
```
Initialize the battery manager, configure ADC and GPIO pins.

### Measurement

```cpp
esp_err_t measure(void);
```
Perform a complete measurement cycle (voltage, charging state, power state).

### Getting Data

```cpp
uint16_t getVoltage(void) const;          // Filtered voltage (mV)
uint8_t getPercentage(void) const;        // Filtered percentage (0-100)
uint16_t getRawVoltage(void) const;       // Unfiltered voltage (mV)
uint8_t getRawPercentage(void) const;     // Unfiltered percentage (0-100)
bool isCharging(void) const;              // Charging state
bool isConnected(void) const;             // Power connection state
battery_state_t getState(void) const;     // Combined state enum
const char* getStateString(void) const;   // State as string
```

### Utility Methods

```cpp
bool isCriticallyLow(uint16_t threshold_mv = 3300);  // Check low battery
bool isFullyCharged(uint8_t threshold_percent = 95); // Check fully charged
void clearFilter(void);                              // Reset filter history
void setFilterSamples(uint8_t samples);              // Change filter size
esp_err_t getReport(char* buff);                     // Get full report
```

### States

```cpp
typedef enum {
    BATTERY_STATE_DISCONNECTED,  // On battery power only
    BATTERY_STATE_CONNECTED,     // Connected to power, not charging
    BATTERY_STATE_CHARGING       // Connected and actively charging
} battery_state_t;
```

## Filter Behavior

The moving average filter smooths voltage and percentage readings:

- **Filter Size**: Configurable (1-20 samples, default: 5)
- **Effect**: Reduces noise and fluctuations
- **Trade-off**: More samples = smoother readings but slower response to changes
- **Recommendation**: 
  - Use 5-7 samples for general use
  - Use 1-3 samples if you need fast response
  - Use 10+ samples if readings are very noisy

## Typical Scenarios

### 1. Monitoring Battery While Disconnected
```cpp
battery.measure();
if (battery.getState() == BATTERY_STATE_DISCONNECTED) {
    uint8_t charge = battery.getPercentage();
    if (charge < 20) {
        printf("Low battery warning: %u%%\n", charge);
    }
}
```

### 2. Detecting Charging Complete
```cpp
battery.measure();
if (battery.isFullyCharged(98)) {
    printf("Battery fully charged, safe to disconnect\n");
}
```

### 3. Power Loss Detection
```cpp
static bool was_connected = true;
battery.measure();

if (was_connected && !battery.isConnected()) {
    printf("Power lost! Running on battery\n");
    // Enter power-saving mode
}
was_connected = battery.isConnected();
```

### 4. Adaptive Behavior Based on State
```cpp
battery.measure();
switch (battery.getState()) {
    case BATTERY_STATE_CHARGING:
        // Can perform intensive operations
        performOTAUpdate();
        break;
        
    case BATTERY_STATE_CONNECTED:
        // Connected but not charging (maybe full)
        normalOperation();
        break;
        
    case BATTERY_STATE_DISCONNECTED:
        // On battery - conserve power
        enablePowerSaving();
        if (battery.isCriticallyLow()) {
            enterDeepSleep();
        }
        break;
}
```

## Calibration

The library uses ESP32 ADC calibration when available (eFuse or Two Point). If calibration is not available, it falls back to raw readings.

Check calibration status:
```cpp
battery.measure();
char report[256];
battery.getReport(report);
printf("%s\n", report);  // Shows "Calibration: Enabled/Disabled"
```

## Troubleshooting

### Readings are unstable
- Increase filter samples: `battery.setFilterSamples(10);`
- Check wiring and connections
- Verify voltage divider resistor values

### Percentage always shows 0% or 100%
- Verify `min_voltage` and `max_voltage` settings match your battery
- Check voltage divider ratio
- Ensure ADC attenuation is correct for your voltage range

### Charging/Connected states incorrect
- Verify GPIO pin assignments
- Check pin logic levels (HIGH = active)
- Test pins with multimeter

### Voltage reads too high/low
- Check voltage divider resistors (R1 and R2)
- Verify ADC attenuation setting
- Recalculate divider ratio: `Vin = Vout * (R1 + R2) / R2`

## Example Output

```
====================================
  Battery Monitoring System
====================================
Iteration 0 | State: Charging
  Voltage:    4174 mV (raw: 4178 mV)
  Charge:     97% (raw: 97%)
  Charging:   YES
  Connected:  YES
------------------------------------
Iteration 1 | State: Charging
  Voltage:    4176 mV (raw: 4174 mV)
  Charge:     97% (raw: 97%)
  Charging:   YES
  Connected:  YES
  ✓ Battery fully charged
------------------------------------
```

## License

© 2026 Sense AI. All rights reserved.

## Version History

- **v1.0.0** (2026-03-16): Initial release
  - Voltage and percentage monitoring
  - Charging and power connection detection
  - Moving average filter
  - State-aware operation

## Support

For questions or issues, please contact: emmanuel@sense-ai.co
