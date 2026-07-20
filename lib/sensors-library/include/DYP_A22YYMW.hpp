/*******************************************************************************
 * @file DYP_A22YYMW.hpp
 * @brief Contains the declarations of the DYP_A22YYMW class methods.
 *
 * A22 PWM mode: measure the Echo time width and convert to distance.
 * This sensor has 2 digital pins: one receives a trigger (input) and the other
 * generates the echo (output), to be readed by the microcontroller.
 *
 * @version v0.1.1
 * @date 2025-15-05
 * @author Isabella Garcia isa@sense-ai.co
 *******************************************************************************/

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "sensors_sense.hpp"

/**
 * @class DYP_A22YYMW
 * @brief A class to interface with the DYP_A22YYMW ultrasonic sensor.
 */
class DYP_A22YYMW : public Sensor {
public:
    /**
     * @brief Class constructor
     *
     * @param trigPin GPIO pin for the trigger signal.
     * @param echoPin GPIO pin for the echo signal.
     */
    DYP_A22YYMW(gpio_num_t trigPin, gpio_num_t echoPin);

    /**
     * @brief Class destructor
     */
    ~DYP_A22YYMW();

    /**
     * @brief Initializes the sensor hardware.
     *
     * @return esp_err_t ESP_OK if successful, or an error code otherwise.
     */
    esp_err_t init(void) override;

    /**
     * @brief Measures the sensor data.
     *
     * @return esp_err_t ESP_OK if successful, or an error code otherwise.
     */
    esp_err_t measure(void) override;

    /**
     * @brief Generates a report of the sensor measurement.
     *
     * @param _buff Buffer to store the report (minimum size: 128 bytes).
     * @return esp_err_t ESP_OK if successful, or an error code otherwise.
     */
    esp_err_t getReport(char* _buff) const override;

    /**
     * @brief Returns the unique ID of the sensor.
     *
     * @return uint8_t Sensor ID.
     */
    uint8_t getID(void) const override;

    /**
     * @brief Reads the distance [mm] measured by the sensor.
     *
     * @param distance Pointer to store the measured distance [mm].
     * @note For continuous sampling, use a 10 ms delay; otherwise, you will get
     * incorrect values.
     * @return esp_err_t ESP_OK if successful, or an error code otherwise.
     */
    esp_err_t readDistance(float* distance) const;

private:
    gpio_num_t trigPin_;  ///< GPIO pin for the trigger signal.
    gpio_num_t echoPin_;  ///< GPIO pin for the echo signal.

    /**
     * @brief Sends the trigger pulse to the sensor.
     */
    void sendPulse() const;
    float distance_ = 0;
};
