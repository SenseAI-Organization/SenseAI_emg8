/*******************************************************************************
 * @file LSM6DSOX_Gyro_ReedSwitch_DataLogger.cpp
 * @brief Data Logger example combining LSM6DSOX (Accel + Gyro) and Reed Switch.
 *
 * This example demonstrates:
 * - Configuring LSM6DSOX Accelerometer and Gyroscope at 208Hz.
 * - Using a High-Resolution Timer (esp_timer) to sample at exactly 100Hz.
 * - Synchronized sampling of Reed Switch state, Accelerometer, and Gyroscope data.
 * - Buffering 512 samples in RAM.
 * - Printing the buffered data in CSV format.
 *
 * @version 0.1.0
 * @date 2025-12-01
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************/

#include <vector>

#include "LSM6DSOX.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "switch_sense.hpp"

// --- Configuration ---
constexpr gpio_num_t kSDA = GPIO_NUM_5;
constexpr gpio_num_t kSCL = GPIO_NUM_4;
constexpr gpio_num_t kReedSwitchPin = GPIO_NUM_17;

constexpr uint32_t kSampleRateHz = 100;
constexpr uint32_t kSamplePeriodUs = 1000000 / kSampleRateHz;
constexpr size_t kBufferSize = 512;

// --- Objects ---
I2C i2c1(I2C_NUM_1, kSDA, kSCL, 400000, false);
Switch reedSwitch(kReedSwitchPin, Switch::SwitchMode::kNormallyClosed, false);

// --- Data Structure ---
struct DataPoint {
    bool switchState;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};

// --- Globals ---
std::vector<DataPoint> dataBuffer;
SemaphoreHandle_t bufferFullSemaphore;
esp_timer_handle_t sampleTimer;

// --- Timer Callback (ISR context) ---
void IRAM_ATTR onTimer(void* arg) {
    // Notify the task to sample
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(arg), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// --- Sampling Task ---
void samplingTask(void* pvParameters) {
    // Reserve memory
    dataBuffer.reserve(kBufferSize);

    // Create LSM6DSOX instance
    LSM6DSOX imu(i2c1, LSM6DSOX::kAddressLow);

    // Init Sensor
    if (imu.init() != ESP_OK) {
        printf("Failed to init LSM6DSOX\n");
        vTaskDelete(nullptr);
    }

    // Configure Accel: 208Hz (Nyquist > 100Hz)
    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k4g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k208Hz);

    // Configure Gyro: 208Hz
    imu.configureGyroFullScale(LSM6DSOX::GyroFullScale::k2000dps);
    imu.configureGyroDataRate(LSM6DSOX::GyroOutputDataRate::k208Hz);

    printf("Sensor Configured. Starting Sampling at %lu Hz...\n", kSampleRateHz);

    // Start Timer
    esp_timer_create_args_t timerArgs = {};
    timerArgs.callback = &onTimer;
    timerArgs.arg = xTaskGetCurrentTaskHandle();
    timerArgs.name = "sample_timer";

    ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &sampleTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sampleTimer, kSamplePeriodUs));

    while (true) {
        // Wait for timer trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 1. Sample Reed Switch (GPIO Read is fast)
        bool swState = reedSwitch.readState();

        // 2. Sample IMU (I2C Transaction)
        // measure() reads the registers into internal buffer
        imu.measure();

        int16_t rawAccel[3];
        int16_t rawGyro[3];
        imu.getAccelRawData(rawAccel);
        imu.getGyroRawData(rawGyro);

        // 3. Store
        if (dataBuffer.size() < kBufferSize) {
            dataBuffer.push_back({swState, rawAccel[0], rawAccel[1], rawAccel[2],
                                  rawGyro[0], rawGyro[1], rawGyro[2]});
        }

        // 4. Check if full
        if (dataBuffer.size() >= kBufferSize) {
            // Stop timer temporarily to print
            esp_timer_stop(sampleTimer);

            printf("\n--- Buffer Full ---\n");
            printf("Switch, Ax, Ay, Az, Gx, Gy, Gz\n");
            for (size_t i = 0; i < dataBuffer.size(); i++) {
                printf("%d, %d, %d, %d, %d, %d, %d\n", dataBuffer[i].switchState,
                       dataBuffer[i].ax, dataBuffer[i].ay, dataBuffer[i].az,
                       dataBuffer[i].gx, dataBuffer[i].gy, dataBuffer[i].gz);
            }

            printf("--- End of Data ---\n");

            // Clear and Resume
            dataBuffer.clear();
            vTaskDelay(pdMS_TO_TICKS(100));  // Short pause
            esp_timer_start_periodic(sampleTimer, kSamplePeriodUs);
        }
    }
}

extern "C" void app_main() {
    printf("\n\n<< LSM6DSOX (Accel+Gyro) + Reed Switch Data Logger >>\n\n");

    // Init I2C
    ESP_ERROR_CHECK(i2c1.init());

    // Init Switch
    ESP_ERROR_CHECK(reedSwitch.init());

    // Start Sampling Task
    // Priority should be high to minimize jitter
    xTaskCreate(samplingTask, "Sampler", 4096, nullptr, configMAX_PRIORITIES - 1,
                nullptr);
}
