/*******************************************************************************
 * @file TurbineFlowMeter.hpp
 * @brief Contains the declarations of the TurbineFlowMeter class methods.
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

#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sensors_sense.hpp"

/**
 * @class TurbineFlowMeter
 * @brief Class that allows interaction with digital turbine flow sensors.
 * It only works on turbines with 1 pulse/revolution.
 * Just provide the pin and KcalibrationFactor. Library runs by itself
 * couting interrupts. UpdateFlow() has to be called at least 1 time/s.
 */
class TurbineFlowMeter : public Sensor {
public:
    /**
     * @brief Constructor for the TurbineFlowMeter class.
     * @param flowSensorPin GPIO pin number.
     * @param calibrationFactor Kfactor [pulses/L] ¿how many pulses corresponds
     * to 1L?
     */
    TurbineFlowMeter(gpio_num_t flowSensorPin, float calibrationFactor);

    /**
     * @brief Destructor for the TurbineFlowMeter class.
     */
    ~TurbineFlowMeter();

    /**
     * @brief Configures GPIO. Starts the interrupt servise rutine and the pulse
     * counter task on Core 0.
     */
    esp_err_t init(void) override;

    uint8_t getID(void) const override;

    /**
     * @brief Call this function at least 1 time/s, based on the pulse counter
     * calcultes all the output values.
     * @return ESP_OK if new data is ready. ESP_ERR_NOT_FINISHED if data hasn't
     * been converted yet.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Preferably call this function after updateFlow() returns TRUE.
     * @return float, flow value [L/min].
     */
    float getFlowRate() const;

    /**
     * @brief Preferably call this function after updateFlow() returns TRUE.
     * @return float, total litres value [L/min] since init() was called.
     */
    float getTotalLitres() const;

    /**
     * @brief Get the K factor.
     * @return float.
     */
    float getCalibrationFactor() const;

    /**
     * @brief Get the instantaneus pulse count in that interval [pulses/second].
     * @return int.
     */
    uint16_t getPulseCount() const;

    /**
     * @brief Get the accumulative pulse count [pulses] since init() was called.
     * @return int.
     */
    uint16_t getPulseCountSum() const;

    /**
     * @brief Get the current time, since init() was called.
     * @return int, milliseconds.
     */
    uint64_t getCurrentMillis() const;

    /**
     * @brief Sets to zero the total liters variable.
     */
    void resetTotalLitres();

private:
    gpio_num_t flowSensorPin_;
    float calibrationFactor_;
    uint64_t oldTime_;
    float flowRate_;
    float totalLitres_;
    volatile uint16_t pulseCount_;  // instaneus pulse count
    uint16_t lastPulseCount_;  // previous pulse count (from the last interval).
    uint16_t lastPulseCountSum_;  // previous pulse count sumatory
    uint16_t pulseCountSum_ = 0;  // actual pulse count sumatory
    const uint8_t kSensorID_ = 0x02;

    static QueueHandle_t s_gpioEventQueue;  // Shared queue for button events
    static void taskFunction(void* pvParameters);
    static void IRAM_ATTR pulseCounter(void* arg);
    static bool s_isFirstInstance;  // initialize once the task function
    void resetPulseCount();
    esp_err_t getReport(char* _buff) const override;
};
