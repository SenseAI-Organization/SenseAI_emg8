/*******************************************************************************
 * @file sensors_sense.cpp
 * @brief Contains the definitions to work with the Sensor, Digital Sensor and
 * Analog Sensor classes.
 *
 * @version 0.3.0
 * @date 2025-08-01
 * @author emmanuel@sense-ai.co, Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "sensors_sense.hpp"

Sensor::~Sensor() {
}

/******************************************************************************/
/*                                  Digital Sensor                            */
/******************************************************************************/

DigitalSensor::DigitalSensor(gpio_num_t pinNumber, PinResistorMode resistor) {
    pinNumber_ = pinNumber;
    sensorConfig_.pin_bit_mask = (1U << pinNumber);
    setPinInternalResistor(resistor);
}

DigitalSensor::~DigitalSensor() {
    gpio_isr_handler_remove(pinNumber_);
    gpio_reset_pin(pinNumber_);
    vSemaphoreDelete(semaphore_);
}

esp_err_t DigitalSensor::init(void) {
    return gpio_config(&sensorConfig_);
}

esp_err_t DigitalSensor::measure(void) {
    previousState_ = pinState_;
    pinState_ = static_cast<int>(gpio_get_level(pinNumber_));
    return ESP_OK;
}

gpio_config_t DigitalSensor::getPinConfiguration(void) const {
    return sensorConfig_;
}

esp_err_t DigitalSensor::setPinNumber(gpio_num_t pinNumber) {
    if (pinNumber < GPIO_NUM_0 || pinNumber > GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    pinNumber_ = pinNumber;
    sensorConfig_.pin_bit_mask = (1U << pinNumber);
    return ESP_OK;
}

gpio_num_t DigitalSensor::getPinNumber(void) const {
    return pinNumber_;
}

esp_err_t DigitalSensor::setPinInternalResistor(PinResistorMode resistorConfig) {
    esp_err_t err = ESP_OK;
    switch (resistorConfig) {
        case DISABLE_RESISTOR:
            sensorConfig_.pull_up_en = GPIO_PULLUP_DISABLE;
            sensorConfig_.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        case PULL_UP:
            sensorConfig_.pull_up_en = GPIO_PULLUP_ENABLE;
            sensorConfig_.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        case PULL_DOWN:
            sensorConfig_.pull_up_en = GPIO_PULLUP_DISABLE;
            sensorConfig_.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        default:  // Error
            err = ESP_ERR_INVALID_ARG;
            break;
    }
    return err;
}

PinResistorMode DigitalSensor::getPinInternalResistor(void) const {
    if (sensorConfig_.pull_up_en) {
        return PULL_UP;
    } else if (sensorConfig_.pull_down_en) {
        return PULL_DOWN;
    } else {
        return DISABLE_RESISTOR;
    }
}

esp_err_t DigitalSensor::setPinInterrupt(gpio_int_type_t interruptType) {
    if (interruptType < GPIO_INTR_DISABLE || interruptType > GPIO_INTR_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorConfig_.intr_type = interruptType;
    return ESP_OK;
}

int8_t DigitalSensor::getPinState(void) const {
    return pinState_;
}

int8_t DigitalSensor::getPreviousPinState(void) const {
    return previousState_;
}

esp_err_t DigitalSensor::configureInterrupt(gpio_int_type_t intrType,
                                            DigitalSensorCallback callback, void* arg) {
    callback_ = callback;
    callbackArg_ = arg;
    sensorConfig_.intr_type = intrType;

    esp_err_t err = gpio_set_intr_type(pinNumber_, sensorConfig_.intr_type);
    if (err) {
        return err;
    }

    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err) {
        return err;
    }

    err = gpio_isr_handler_add(pinNumber_, DigitalSensor::isr_handler, this);
    if (err) {
        return err;
    }

    semaphore_ = xSemaphoreCreateBinary();

    return err;
}

void DigitalSensor::startHandlerTask(const char* taskName, UBaseType_t taskPriority,
                                     int stackSize) {
    xTaskCreate(
        [](void* arg) { static_cast<DigitalSensor*>(arg)->handleSwitchActuation(); },
        taskName, stackSize, this, taskPriority, nullptr);
}

IRAM_ATTR void DigitalSensor::isr_handler(void* arg) {
    DigitalSensor* sensor = static_cast<DigitalSensor*>(arg);
    uint64_t currentTime = esp_timer_get_time();

    const uint64_t debounceDelay = 500 * 1000;
    if (currentTime - sensor->lastInterruptTime_ > debounceDelay) {
        sensor->lastInterruptTime_ = currentTime;
        xSemaphoreGiveFromISR(sensor->semaphore_, NULL);
    }
}

void DigitalSensor::handleSwitchActuation(void) {
    while (true) {
        if (xSemaphoreTake(semaphore_, portMAX_DELAY) == pdTRUE) {
            if (callback_) {
                callback_(callbackArg_);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/******************************************************************************/
/*                                  Analog Sensor                             */
/******************************************************************************/
std::map<adc_unit_t, adc_oneshot_unit_handle_t> AnalogSensor::s_unitHandles;
std::map<AnalogSensor::CaliKey, adc_cali_handle_t> AnalogSensor::s_caliHandles;

AnalogSensor::AnalogSensor(adc_channel_t channel) : channel_(channel) {
    adc_oneshot_channel_to_io(unitConfig_.unit_id, channel_,
                              reinterpret_cast<int*>(&pin_));
}

AnalogSensor::AnalogSensor(gpio_num_t pin) : pin_(pin) {
    adc_oneshot_io_to_channel(static_cast<int>(pin_), &unitConfig_.unit_id, &channel_);
}

AnalogSensor::AnalogSensor(adc_channel_t channel, adc_atten_t attenuation,
                           adc_bitwidth_t resolution)
    : channel_(channel) {
    channelConfig_.atten = attenuation;
    channelConfig_.bitwidth = resolution;
    adc_oneshot_channel_to_io(unitConfig_.unit_id, channel_,
                              reinterpret_cast<int*>(&pin_));
}

AnalogSensor::AnalogSensor(gpio_num_t pin, adc_atten_t attenuation,
                           adc_bitwidth_t resolution)
    : pin_(pin) {
    adc_oneshot_io_to_channel(static_cast<int>(pin_), &unitConfig_.unit_id, &channel_);
    channelConfig_.atten = attenuation;
    channelConfig_.bitwidth = resolution;
}

AnalogSensor::~AnalogSensor() {
    gpio_reset_pin(pin_);
}

esp_err_t AnalogSensor::init() {
    if (s_unitHandles.count(unitConfig_.unit_id) == 0) {
        adc_oneshot_unit_handle_t newHandle;
        esp_err_t err = adc_oneshot_new_unit(&unitConfig_, &newHandle);
        if (err != ESP_OK) {
            return err;
        }

        s_unitHandles[unitConfig_.unit_id] = newHandle;
    }

    adcHandle_ = s_unitHandles[unitConfig_.unit_id];

    CaliKey caliKey = {
        .unit = unitConfig_.unit_id,
        .atten = channelConfig_.atten,
        .bitwidth = channelConfig_.bitwidth,
    };

    if (s_caliHandles.count(caliKey) == 0) {
        adc_cali_handle_t newCaliHandle = nullptr;
        adc_cali_curve_fitting_config_t caliConfig = {
            .unit_id = caliKey.unit,
            .atten = caliKey.atten,
            .bitwidth = caliKey.bitwidth,
        };

        esp_err_t err = adc_cali_create_scheme_curve_fitting(&caliConfig, &newCaliHandle);

        s_caliHandles[caliKey] = newCaliHandle;
    }

    caliHandle_ = s_caliHandles[caliKey];

    return adc_oneshot_config_channel(adcHandle_, channel_, &channelConfig_);
}

esp_err_t AnalogSensor::measure() {
    previousValue_ = adcValue_;
    calibratedVal_ = kAdcValueError;  // Reset calibrated value

    if (!adcHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = adc_oneshot_read(adcHandle_, channel_, &adcValue_);
    if (err != ESP_OK) {
        return err;
    }

    if (useCalibration_ && caliHandle_) {
        adc_cali_raw_to_voltage(caliHandle_, adcValue_, &calibratedVal_);
    }

    return ESP_OK;
}

esp_err_t AnalogSensor::configure(void) {
    channelConfig_.atten = ADC_ATTEN_DB_12;
    channelConfig_.bitwidth = ADC_BITWIDTH_DEFAULT;
    return ESP_OK;
}

esp_err_t AnalogSensor::configure(adc_channel_t channel, adc_atten_t attenuation,
                                  adc_bitwidth_t resolution) {
    channel_ = channel;
    channelConfig_.atten = attenuation;
    channelConfig_.bitwidth = resolution;
    return adc_oneshot_channel_to_io(unitConfig_.unit_id, channel_,
                                     reinterpret_cast<int*>(&pin_));
}

esp_err_t AnalogSensor::configure(gpio_num_t pin, adc_atten_t attenuation,
                                  adc_bitwidth_t resolution) {
    pin_ = pin;
    channelConfig_.atten = attenuation;
    channelConfig_.bitwidth = resolution;

    return adc_oneshot_io_to_channel(static_cast<int>(pin_), &unitConfig_.unit_id,
                                     &channel_);
}

esp_err_t AnalogSensor::configure(adc_oneshot_unit_init_cfg_t unitConfig,
                                  adc_channel_t channel,
                                  adc_oneshot_chan_cfg_t channelConfig) {
    unitConfig_ = unitConfig;
    channel_ = channel;
    channelConfig_ = channelConfig;

    return adc_oneshot_channel_to_io(unitConfig_.unit_id, channel_,
                                     reinterpret_cast<int*>(&pin_));
}

esp_err_t AnalogSensor::configure(adc_oneshot_unit_init_cfg_t unitConfig, gpio_num_t pin,
                                  adc_oneshot_chan_cfg_t channelConfig) {
    unitConfig_ = unitConfig;
    pin_ = pin;
    channelConfig_ = channelConfig;

    return adc_oneshot_io_to_channel(static_cast<int>(pin_), &unitConfig_.unit_id,
                                     &channel_);
}

int AnalogSensor::getValue() const {
    return adcValue_;
}

int AnalogSensor::getCalibratedMv() const {
    return calibratedVal_;
}

int AnalogSensor::read() {
    measure();
    return getValue();
}

int AnalogSensor::readCalibrated() {
    measure();
    return getCalibratedMv();
}

int AnalogSensor::getPreviousValue() const {
    return previousValue_;
}

int AnalogSensor::getAdcMaxValue() const {
    return adcMaxValue_;
}

int AnalogSensor::getVRef() const {
    return vRef_;
}

gpio_num_t AnalogSensor::getPinNumber() const {
    return pin_;
}

adc_channel_t AnalogSensor::getChannelNumber(void) const {
    return channel_;
}

esp_err_t AnalogSensor::setAttenuation(adc_atten_t attenuation) {
    if (attenuation < ADC_ATTEN_DB_0 || attenuation > ADC_ATTEN_DB_12) {
        return ESP_ERR_INVALID_ARG;
    }

    channelConfig_.atten = attenuation;
    setVRef(channelConfig_.atten);
    return ESP_OK;
}

adc_atten_t AnalogSensor::getAttenuation(void) const {
    return channelConfig_.atten;
}

esp_err_t AnalogSensor::setBitResolution(adc_bitwidth_t resolution) {
    if (resolution < ADC_BITWIDTH_DEFAULT || resolution > ADC_BITWIDTH_13) {
        return ESP_ERR_INVALID_ARG;
    }

    channelConfig_.bitwidth = resolution;

    if (channelConfig_.bitwidth) {
        adcMaxValue_ = (1U << channelConfig_.bitwidth) - 1;
    } else {
        adcMaxValue_ = (1U << ADC_BITWIDTH_12) - 1;
    }

    return ESP_OK;
}

adc_bitwidth_t AnalogSensor::getBitResolution(void) const {
    return channelConfig_.bitwidth;
}

void AnalogSensor::setVRef(adc_atten_t atten) {
    switch (atten) {
        case ADC_ATTEN_DB_0:
            vRef_ = 1100;
            break;
        case ADC_ATTEN_DB_2_5:
            vRef_ = 1463;  // vref * 1.33f;
            break;
        case ADC_ATTEN_DB_6:
            vRef_ = 2200;  // vref * 2.0f;
            break;
        case ADC_ATTEN_DB_12:
            vRef_ = 3905;  // vref * 3.55f;
            break;
        default:
            vRef_ = 3905;
            break;
    }
}

esp_err_t AnalogSensor::configureAdcUnit(adc_oneshot_unit_init_cfg_t unitConfig) {
    unitConfig_ = unitConfig;
    return ESP_OK;
}

esp_err_t AnalogSensor::configureAdcChannel(adc_oneshot_chan_cfg_t channelConfig) {
    channelConfig_ = channelConfig;
    return ESP_OK;
}

void AnalogSensor::enableCalibration(bool enable) {
    useCalibration_ = enable;
}

bool AnalogSensor::isCalibrationEnabled(void) const {
    return (caliHandle_ != nullptr) && useCalibration_;
}
