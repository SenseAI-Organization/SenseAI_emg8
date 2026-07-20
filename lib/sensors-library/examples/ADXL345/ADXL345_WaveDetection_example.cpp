/*******************************************************************************
 * @file ADXL345_WaveDetection_example.cpp
 * @brief File with an example on how to detect waves using the ADXL345 and
 * sampling the measures with timer interrupts.
 *
 * @version 0.1.1
 * @date 2025-03-11
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include <cmath>

#include "ADXL345.hpp"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "smart_sensor_sense.hpp"
#include "esp_log.h"

// Global flag for timer-based sampling
volatile bool sampleFlag = false;

// Z-axis detection parameters
constexpr uint8_t kZBufferSize = 8;
constexpr float kZHysteresisThreshold = 5.0f;
constexpr uint8_t kConsecutiveCountRequired = 16;

static int16_t zBuffer[kZBufferSize] = {0};
static uint8_t zBufferIndex = 0;
static float sumForAverageZ = 0.0f;
static uint8_t consecutiveBelowThreshold = 0;

// ADXL345 interrupt pin
constexpr gpio_num_t kInt1_Pin = GPIO_NUM_19;

// Function prototypes
void resetWaveDetection(void);
bool detectWaveEnd_z(int16_t currentZ);
void captureWaveEvent(ADXL345& accel);
esp_err_t configureAdxl345(ADXL345& adxl345);

// Timer callback
void IRAM_ATTR timerCallback(void* arg) {
    sampleFlag = true;
}

extern "C" void app_main() {
    uart_set_baudrate(UART_NUM_0, 921600);
    esp_log_level_set("*", ESP_LOG_NONE);
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("\n\n<< Hybrid Wave Capture with Light Sleep >>\n\n");

    const gpio_num_t kSDA1 = GPIO_NUM_5;
    const gpio_num_t kSCL1 = GPIO_NUM_4;
    I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);
    ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);

    esp_err_t err = i2c1.init();
    if (err != ESP_OK) {
        printf("Error initializing I2C: %s\n", esp_err_to_name(err));
        while (true);
    }
    err = accel_i2c.init();
    if (err != ESP_OK) {
        printf("Error initializing ADXL345: %s\n", esp_err_to_name(err));
        while (true);
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

    // Setup timer with a 3 ms period
    Timer timer(3);
    if (timer.init() != ESP_OK || timer.configure() != ESP_OK) {
        printf("Failed to initialize/configure timer.\n");
        return;
    }

    timer.setCallback(timerCallback);
    timer.start();

    // Configure light sleep wakeup on kInt1_Pin (active high)
    esp_sleep_enable_ext0_wakeup(kInt1_Pin, 1);

    while (true) {
        printf("Entering light sleep...\n");
        esp_light_sleep_start();

        if (accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity)) {
            printf("ACTIVITY DETECTED!\n");
            captureWaveEvent(accel_i2c);
            accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity);
        }
    }
}

void resetWaveDetection(void) {
    for (uint8_t i = 0; i < kZBufferSize; i++) {
        zBuffer[i] = 0;
    }
    sumForAverageZ = 0.0f;
    consecutiveBelowThreshold = 0;
    zBufferIndex = 0;
}

bool detectWaveEnd_z(int16_t currentZ) {
    sumForAverageZ += (currentZ - zBuffer[zBufferIndex]);
    zBuffer[zBufferIndex] = currentZ;
    zBufferIndex = (zBufferIndex + 1) % kZBufferSize;

    float rollingAvg = sumForAverageZ / kZBufferSize;
    if (fabs(currentZ - rollingAvg) < kZHysteresisThreshold) {
        consecutiveBelowThreshold++;
    } else {
        consecutiveBelowThreshold = 0;
    }
    if (consecutiveBelowThreshold >= kConsecutiveCountRequired) {
        consecutiveBelowThreshold = 0;
        for (uint8_t i = 0; i < kZBufferSize; i++) {
            zBuffer[i] = 0;
        }
        sumForAverageZ = 0.0f;
        return true;
    }
    return false;
}

void captureWaveEvent(ADXL345& accel) {
    resetWaveDetection();
    const uint16_t kMaxWaveSize = 512;
    int16_t waveBuffer[kMaxWaveSize] = {0};
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
            int16_t currentZ = data[2];
            waveBuffer[waveSampleCount++] = currentZ;
            if (detectWaveEnd_z(currentZ)) break;
            if (waveSampleCount >= kMaxWaveSize) break;
        }
    }

    printf("Wave Event Captured: %u samples\n", waveSampleCount);
    for (uint16_t i = 0; i < waveSampleCount; i++) {
        printf("%i\n", waveBuffer[i]);
    }
}

esp_err_t configureAdxl345(ADXL345& adxl345) {
    esp_err_t err = adxl345.disableMeasure();
    err |= adxl345.configureOutPutDataRate(ADXL345::OutputDataRate::k800Hz);
    const uint8_t kThreshold_0_3_G = 0x05;
    err |= adxl345.configureActivityThreshold(kThreshold_0_3_G);
    err |= adxl345.configureActivityControl(ADXL345::ActivityControl::kActZ |
                                            ADXL345::ActivityControl::kActAcDc);
    err |= adxl345.assignInterrupts(ADXL345::InterruptFlags::kActivity,
                                    ADXL345::Interrupt::INT1);
    err |= adxl345.configurePin(kInt1_Pin, ADXL345::Interrupt::INT1, ActiveLevel::kHigh,
                                false);
    err |= adxl345.enableInterrupts(ADXL345::InterruptFlags::kActivity);
    err |= adxl345.enableMeasure();
    return err;
}
