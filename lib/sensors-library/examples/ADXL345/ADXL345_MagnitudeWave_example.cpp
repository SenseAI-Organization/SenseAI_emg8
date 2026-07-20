/*******************************************************************************
 * @file ADXL345_MagnitudeWave_example.cpp
 * @brief Detects waves on the 3 axes using the ADXL345 and timer interrupts.
 *        Prints both the computed magnitude and the raw 3-axis data.
 * @version 0.1.1
 * @date 2025-03-11
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include <cmath>

#include "driver/uart.h"
#include "esp_sleep.h"

#include "ADXL345.hpp"
#include "smart_sensor_sense.hpp"

// Global flag for timer-based sampling
volatile bool sampleFlag = false;

// ADXL345 interrupt pin
constexpr gpio_num_t kInt1_Pin = GPIO_NUM_19;

// Magnitude detection parameters
constexpr uint8_t kBufferSize = 8;
constexpr float kHysteresisThreshold = 3.0f;  // Adjust based on application
constexpr uint8_t kConsecutiveCountRequired = 5;

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

// Timer callback (3 ms period)
void IRAM_ATTR timerCallback(void* arg) {
    sampleFlag = true;
}

// Main application entry point
extern "C" void app_main(void) {
    uart_set_baudrate(UART_NUM_0, 921600);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n\n<< Hybrid Wave Capture with Light Sleep program started>>\n\n");

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

    err = accel_i2c.init();
    if (err != ESP_OK) {
        printf("Error initializing ADXL345: %s\n", esp_err_to_name(err));

        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    printf("Calibrating ADXL345 sensor. Do not move the sensor...\n");
    err = accel_i2c.calibrate();
    if (err != ESP_OK) {
        printf("Error calibrating ADXL345: %s\n", esp_err_to_name(err));

        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    err = configureAdxl345(accel_i2c);
    if (err != ESP_OK) {
        printf("Error configuring ADXL345: %s\n", esp_err_to_name(err));

        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    Timer timer(3);

    if (timer.init() != ESP_OK || timer.configure() != ESP_OK) {
        printf("Failed to initialize timer.\n");
        return;
    }

    timer.setCallback(timerCallback);
    timer.start();

    esp_sleep_enable_ext0_wakeup(kInt1_Pin, 1);

    while (true) {
        printf("Entering light sleep...\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_light_sleep_start();

        if (accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity)) {
            printf("Wave!\n");
            captureWaveEvent(accel_i2c);
            // Clear the interrupt flag if needed
            accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity);
        }
    }
}

// Reset rolling average data
void resetWaveDetection(void) {
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
    resetWaveDetection();
    uint16_t waveSampleCount = 0;
    int16_t data[3] = {0};

    while (true) {
        if (sampleFlag) {
            sampleFlag = false;

            esp_err_t err = accel.measure();
            if (err != ESP_OK) {
                printf("Error measuring ADXL345: %d\n", err);
            }
            
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

            if (detectWaveEnd(currentMagnitude)) {
                break;
            }

            if (waveSampleCount >= kMaxWaveSize) {
                break;
            }
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
    err |= adxl345.configureOutPutDataRate(ADXL345::OutputDataRate::k800Hz);
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
    err |= adxl345.configurePin(kInt1_Pin, ADXL345::Interrupt::INT1,
                                ActiveLevel::kHigh, false);
    err |= adxl345.enableInterrupts(ADXL345::InterruptFlags::kActivity);
    err |= adxl345.enableMeasure();
    return err;
}