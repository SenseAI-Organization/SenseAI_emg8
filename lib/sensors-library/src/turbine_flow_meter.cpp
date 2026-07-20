/*******************************************************************************
 * @file TurbineFlowMeter.cpp
 * @brief Contains the definitions of the TurbineFlowMeter class methods.
 *
 * This sensor generates pulses from the water that passes through it.
 * It only works on turbines with 1 pulse/revolution.
 * Library runs by itself counting interrupts. UpdateFlow() has to be called at
 * least 1 time/s.
 *
 * @version v0.1.1
 * @date 2025-15-05
 * @author mauricio@sense-ai.co, Sense AI
 *******************************************************************************/

#include "turbine_flow_meter.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensors_sense.hpp"

QueueHandle_t TurbineFlowMeter::s_gpioEventQueue = nullptr;
bool TurbineFlowMeter::s_isFirstInstance = false;

TurbineFlowMeter::TurbineFlowMeter(gpio_num_t flowSensorPin,
                                   float calibrationFactor)
    : flowSensorPin_(flowSensorPin),
      calibrationFactor_(calibrationFactor),
      oldTime_(0),
      flowRate_(0.0),
      totalLitres_(0.0),
      pulseCount_(0),
      pulseCountSum_(0) {
}

TurbineFlowMeter::~TurbineFlowMeter() {
}

esp_err_t TurbineFlowMeter::init(void) {
    gpio_config_t ioConf = {};

    ioConf.intr_type = GPIO_INTR_POSEDGE;
    ioConf.pin_bit_mask = (1ULL << flowSensorPin_);
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pull_up_en = GPIO_PULLUP_ENABLE;

    esp_err_t ret = gpio_config(&ioConf);

    if (ret != ESP_OK) {
        return ret;
    }

    if (s_gpioEventQueue == NULL) {
        s_gpioEventQueue = xQueueCreate(20, sizeof(TurbineFlowMeter *));

        if (!s_isFirstInstance) {  // Check if it's the first object
            xTaskCreatePinnedToCore(taskFunction,  // Task function
                                    "flow_task",   // Name of the task
                                    2048,          // Stack size in bytes
                                    NULL,          // Task parameters
                                    9,             // Priority
                                    NULL,          // Task handle
                                    0  // Core ID (0 = Core 0, 1 = Core 1)
            );
            s_isFirstInstance = true;  // Mark that initialization has been done
        }
    }

    gpio_install_isr_service(0);
    ret = gpio_isr_handler_add(flowSensorPin_, pulseCounter, (void *)this);

    return ret;
}

void IRAM_ATTR TurbineFlowMeter::pulseCounter(void *arg) {
    TurbineFlowMeter *sensor = static_cast<TurbineFlowMeter *>(arg);
    xQueueSendFromISR(s_gpioEventQueue, &sensor, NULL);
}

/* Task: Processes button events and updates counters */
void TurbineFlowMeter::taskFunction(void *pvParameters) {  // core0
    TurbineFlowMeter *sensorInstance;

    while (1) {
        if (xQueueReceive(s_gpioEventQueue, &sensorInstance,
                          pdMS_TO_TICKS(2000))) {
            uint16_t temp = sensorInstance->pulseCount_;  // Read value safely
            temp++;                                       // Increment locally
            sensorInstance->pulseCount_ = temp;           // Write back safely
        }
    }
}

esp_err_t TurbineFlowMeter::measure() {
    uint64_t currentMicros = esp_timer_get_time();  // Time in microseconds

    if ((currentMicros - oldTime_) > 1'000'000) {  // 1 second = 1,000,000 µs
        uint64_t deltaMicros = currentMicros - oldTime_;

        flowRate_ = ((1'000'000.0 / float(deltaMicros)) * float(pulseCount_)) /
                    calibrationFactor_ * 60.0;
        totalLitres_ += float(pulseCount_) / calibrationFactor_;
        pulseCountSum_ += pulseCount_;

        oldTime_ = currentMicros;
        lastPulseCount_ = pulseCount_;
        pulseCount_ = 0;

        return ESP_OK;
    } else {
        return ESP_ERR_NOT_FINISHED;
    }
}

void TurbineFlowMeter::resetPulseCount() {
    pulseCount_ = 0;
}

void TurbineFlowMeter::resetTotalLitres() {
    totalLitres_ = 0;
}

float TurbineFlowMeter::getFlowRate() const {
    return flowRate_;
}

float TurbineFlowMeter::getCalibrationFactor() const {
    return calibrationFactor_;
}

uint64_t TurbineFlowMeter::getCurrentMillis() const {
    return oldTime_;
}

float TurbineFlowMeter::getTotalLitres() const {
    return totalLitres_;
}

uint16_t TurbineFlowMeter::getPulseCount() const {
    return lastPulseCount_;
}

uint16_t TurbineFlowMeter::getPulseCountSum() const {
    return pulseCountSum_;
}

esp_err_t TurbineFlowMeter::getReport(char *buff) const {
    if (buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(buff, 128, "Flow rate: %.2f L/min, Total Litres: %.2f L",
             flowRate_, totalLitres_);

    return ESP_OK;
}

uint8_t TurbineFlowMeter::getID(void) const {
    return kSensorID_;
}
