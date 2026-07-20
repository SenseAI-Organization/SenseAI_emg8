/*******************************************************************************
 * @file LSM6DSOX_ReedSwitch_DataLogger.cpp
 * @brief Data Logger example combining LSM6DSOX and Reed Switch.
 *
 * This example demonstrates:
 * - Configuring LSM6DSOX Accelerometer at 208Hz.
 * - Using a High-Resolution Timer (esp_timer) to sample at exactly 100Hz.
 * - Synchronized sampling of Reed Switch state and Accelerometer data.
 * - Buffering 512 samples in RAM.
 * - Printing the buffered data in CSV format.
 *
 * @note Why not use Sensor FIFO?
 *       While the LSM6DSOX FIFO is efficient for accelerometer history, it cannot
 *       automatically record the state of an external GPIO (Reed Switch) synchronized
 *       with each sample. To ensure the Switch state corresponds exactly to the
 *       Accelerometer reading at 100Hz, we use the ESP32 to drive the sampling.
 *
 * @version 0.1.0
 * @date 2025-11-27
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
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
};

// --- Globals ---
std::vector<DataPoint> dataBuffer;
SemaphoreHandle_t bufferFullSemaphore;
esp_timer_handle_t sampleTimer;

// --- Timer Callback (ISR context) ---
void IRAM_ATTR onTimer(void* arg) {
    // Notify the task to sample
    // We use a direct notification or semaphore.
    // Since we need to sample *now* to be accurate, we could sample here,
    // but I2C in ISR is tricky/forbidden.
    // However, at 100Hz, a task notification is fast enough (<1ms latency).

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

    // Create LSM6DSOX instance locally or global? Local is fine if passed or global.
    // We'll use a local pointer to the global i2c for the sensor.
    LSM6DSOX imu(i2c1, LSM6DSOX::kAddressLow);

    // Init Sensor
    if (imu.init() != ESP_OK) {
        printf("Failed to init LSM6DSOX\n");
        vTaskDelete(nullptr);
    }

    // Configure Accel: 208Hz (Nyquist > 100Hz)
    imu.configureAccelFullScale(LSM6DSOX::AccelFullScale::k2g);
    imu.configureAccelDataRate(LSM6DSOX::AccelOutputDataRate::k208Hz);
    imu.disableGyro();  // Not needed

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

        // 2. Sample Accelerometer (I2C Transaction)
        // measure() reads the registers into internal buffer
        imu.measure();

        int16_t raw[3];
        imu.getAccelRawData(raw);

        // 3. Store
        if (dataBuffer.size() < kBufferSize) {
            dataBuffer.push_back({swState, raw[0], raw[1], raw[2]});
        }

        // 4. Check if full
        if (dataBuffer.size() >= kBufferSize) {
            // Stop timer temporarily to print?
            // Or double buffer? For simplicity, we pause, print, resume.
            esp_timer_stop(sampleTimer);

            printf("\n--- Buffer Full ---\n");
            // printf("Index, Switch, Ax, Ay, Az\n");
            // for (size_t i = 0; i < dataBuffer.size(); i++) {
            //     printf("%d, %d, %d, %d, %d\n", i, dataBuffer[i].switchState,
            //            dataBuffer[i].ax, dataBuffer[i].ay, dataBuffer[i].az);
            // }
            printf("Switch, Ax, Ay, Az\n");
            for (size_t i = 0; i < dataBuffer.size(); i++) {
                printf("%d, %d, %d, %d\n", dataBuffer[i].switchState, dataBuffer[i].ax,
                       dataBuffer[i].ay, dataBuffer[i].az);
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
    printf("\n\n<< LSM6DSOX + Reed Switch Data Logger >>\n\n");

    // Init I2C
    ESP_ERROR_CHECK(i2c1.init());

    // Init Switch
    ESP_ERROR_CHECK(reedSwitch.init());

    // Start Sampling Task
    // Priority should be high to minimize jitter
    xTaskCreate(samplingTask, "Sampler", 4096, nullptr, configMAX_PRIORITIES - 1,
                nullptr);
}
