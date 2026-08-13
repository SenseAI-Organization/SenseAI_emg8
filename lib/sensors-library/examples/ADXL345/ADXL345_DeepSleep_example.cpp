/*******************************************************************************
 * @file ADXL345_DeepSleep_example.cpp
 * @brief Detects waves on the 3 axes using the ADXL345. Retreiving data from
 *        FIFO and using deepsleep cycles.
 * @version 0.2.0
 * @date 2025-03-18
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include <cmath>

#include "soc/rtc.h"
#include "driver/uart.h"
#include "esp_sleep.h"

#include "ADXL345.hpp"
#include "smart_sensor_sense.hpp"

// This flag will persist across deep sleep cycles.
RTC_DATA_ATTR bool sensors_calibrated = false;

// ADXL345 interrupt pin
constexpr gpio_num_t kInt1_Pin = GPIO_NUM_19;
// constexpr gpio_num_t kInt2_Pin = GPIO_NUM_20;

// Magnitude detection parameters
constexpr uint8_t kBufferSize = 8;
constexpr float kHysteresisThreshold = 3.5f;  // Adjust based on application
constexpr uint8_t kConsecutiveCountRequired = 5;
constexpr uint8_t kSamplingDelay = 12;

// Rolling average state variables
static float accBuffer[kBufferSize] = {0.0f};
static uint8_t bufferIndex = 0;
static float sumForAverage = 0.0f;
static uint8_t consecutiveBelowThreshold = 0;

// Structure to hold a sample with 3-axis data and computed magnitude
struct WaveSample {
    float x;
    float y;
    float z;
    float magnitude;
};

// Maximum wave size and global wave buffer
constexpr uint16_t kMaxWaveSize = 256;
static WaveSample globalWaveBuffer[kMaxWaveSize];

// Function prototypes
void resetWaveDetection(void);
bool detectWaveEnd(float currentMagnitude);
void captureWaveEvent(ADXL345 &accel);
esp_err_t configureAdxl345(ADXL345& adxl345);
esp_err_t initConfigureAccel(ADXL345& adxl345);

// Main application entry point
extern "C" void app_main(void) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        // First run (cold boot)
        printf("First boot, normal startup.\n");
    }

    // Be sure this are the right SDA/SCL pins!
    const gpio_num_t kSDA1 = GPIO_NUM_5;
    const gpio_num_t kSCL1 = GPIO_NUM_4;
    I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);
    ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);

    esp_err_t err = i2c1.init();
    if (err != ESP_OK) {
        printf("Error initializing I2C: %s\n", esp_err_to_name(err));
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    if (!sensors_calibrated) {
        err = initConfigureAccel(accel_i2c);
        if (err == ESP_OK) {
            sensors_calibrated = true;
        }
    }

    esp_sleep_enable_ext0_wakeup(kInt1_Pin, 1);

    // printf("Reboot and config time: %d\n", (int) esp_timer_get_time());

    while (true) {
        if (accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity)) {
            captureWaveEvent(accel_i2c);
            // Clear the interrupt flag if needed
            accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity);
        }

        printf("Entering deep sleep...\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_deep_sleep_start();
    }
}

// Reset rolling average data
void resetWaveDetection(void) { // Not needed when using deep sleep
    for (uint8_t i = 0; i < kBufferSize; i++) {
        accBuffer[i] = 0.0f;
    }

    sumForAverage = 0.0f;
    consecutiveBelowThreshold = 0;
    bufferIndex = 0;
}

// Update rolling average and check if the wave has ended
bool detectWaveEnd(float currentMagnitude) {
    sumForAverage += (currentMagnitude - accBuffer[bufferIndex]);
    accBuffer[bufferIndex] = currentMagnitude;
    bufferIndex = (bufferIndex + 1) % kBufferSize;

    float rollingAvg = sumForAverage / kBufferSize;
    if (fabs(currentMagnitude - rollingAvg) < kHysteresisThreshold) {
        consecutiveBelowThreshold++;
    } else {
        consecutiveBelowThreshold = 0;
    }

    if (consecutiveBelowThreshold >= kConsecutiveCountRequired) {
        consecutiveBelowThreshold = 0;

        for (uint8_t i = 0; i < kBufferSize; i++) {
            accBuffer[i] = 0.0f;
        }

        sumForAverage = 0.0f;
        return true;
    }

    return false;
}

// Capture a wave event and print the sample data
void captureWaveEvent(ADXL345 &accel) {
    uint16_t waveSampleCount = 0;

    while (true) {
        if (!accel.isIntFlagUp(ADXL345::InterruptFlags::kWatermark)) {
            continue;
        }

        if (accel.measure() != ESP_OK) {
            printf("Error measuring the sensor!\n");
            break;
        }

        int16_t data[3] = {0};
        accel.getRawData(data);

        float x = static_cast<float>(data[0]);
        float y = static_cast<float>(data[1]);
        float z = static_cast<float>(data[2]);
        float currentMagnitude = sqrtf(x * x + y * y + z * z);

        globalWaveBuffer[waveSampleCount].x = x;
        globalWaveBuffer[waveSampleCount].y = y;
        globalWaveBuffer[waveSampleCount].z = z;
        globalWaveBuffer[waveSampleCount].magnitude = currentMagnitude;
        waveSampleCount++;

        if (waveSampleCount > kSamplingDelay &&
            detectWaveEnd(currentMagnitude)) {
            break;
        }

        if (waveSampleCount >= kMaxWaveSize) {
            break;
        }
    }

    // printf("Wave Event Captured: %u samples\n", waveSampleCount);
    for (uint16_t i = 0; i < waveSampleCount; i++) {
        printf("x: %.2f, y: %.2f, z: %.2f, mag: %.2f\n",
               globalWaveBuffer[i].x,
               globalWaveBuffer[i].y,
               globalWaveBuffer[i].z,
               globalWaveBuffer[i].magnitude);
    }
}

// Configure the ADXL345 for activity detection on all three axes
esp_err_t configureAdxl345(ADXL345& adxl345) {
    esp_err_t err = adxl345.disableMeasure();
    err |= adxl345.configureOutPutDataRate(ADXL345::OutputDataRate::k400Hz,
                                           ADXL345::PowerMode::kLow);
    err |= adxl345.configureAccelerationRange(ADXL345::AccelerationRange::k2g);

    const uint8_t kFifoSize = 30;
    err |= adxl345.configureFIFO(ADXL345::FifoMode::kStream,
                                 kFifoSize, ADXL345::Interrupt::INT1);

    constexpr uint8_t kThreshold_0_3_G = 0x05;
    err |= adxl345.configureActivityThreshold(kThreshold_0_3_G);
    err |= adxl345.configureActivityControl(
        ADXL345::ActivityControl::kActX |
        ADXL345::ActivityControl::kActY |
        ADXL345::ActivityControl::kActZ |
        ADXL345::ActivityControl::kActAcDc
    );
    err |= adxl345.assignInterrupts(ADXL345::InterruptFlags::kActivity,
                                    ADXL345::Interrupt::INT1);
    err |= adxl345.assignInterrupts(ADXL345::InterruptFlags::kWatermark,
                                    ADXL345::Interrupt::INT2);
    err |= adxl345.configurePin(kInt1_Pin, ADXL345::Interrupt::INT1,
                                ActiveLevel::kHigh, false);
    err |= adxl345.enableInterrupts(
        ADXL345::InterruptFlags::kActivity | ADXL345::InterruptFlags::kWatermark
    );
    err |= adxl345.enableMeasure();

    return err;
}


esp_err_t initConfigureAccel(ADXL345& adxl345) {
    esp_err_t err = adxl345.init();
    if (err != ESP_OK) {
        err = adxl345.init(); // Second try
    }

    if (err != ESP_OK) {
        printf("Error initializing ADXL345: %s\n", esp_err_to_name(err));
        return err;
    }

    printf("Calibrating ADXL345 sensor. Do not move the sensor...\n");
    err = adxl345.calibrate();
    if (err != ESP_OK) {
        printf("Error calibrating ADXL345: %s\n", esp_err_to_name(err));
        return err;
    }

    err = configureAdxl345(adxl345);
    if (err != ESP_OK) {
        printf("Error configuring ADXL345: %s\n", esp_err_to_name(err));
        return err;
    }

    return err;
}
