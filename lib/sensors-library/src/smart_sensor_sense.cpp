/*******************************************************************************
 * @file smart_sensor_sense.hpp
 * @brief Contains the definitions of the states of the main file.
 *
 * @version v0.2.1
 * @date 2025-24-02
 * @author emmanuel@sense-ai.co, Sense-AI
 *******************************************************************************
 *******************************************************************************/

#include "esp_rom_sys.h"
#include "smart_sensor_sense.hpp"

I2C::I2C(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t frequency,
         bool internalPullup)
    : port_(port),
      sdaPin_(sda_pin),
      sclPin_(scl_pin),
      frequency_(frequency),
      internalResistor_(internalPullup) {
}

I2C::~I2C() {
    deinit();
}

esp_err_t I2C::init() {
    clearBus();
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin_;
    conf.scl_io_num = sclPin_;
    conf.sda_pullup_en = internalResistor_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = internalResistor_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = frequency_;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t err = i2c_param_config(port_, &conf);
    if (err != ESP_OK) {
        printf("I2C parameter configuration failed: %s\n", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        printf("I2C driver installation failed: %s\n", esp_err_to_name(err));
    }

    return err;
}

void I2C::deinit() {
    i2c_driver_delete(port_);
}

esp_err_t I2C::write(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data,
                     size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t I2C::read(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data,
                    size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return err;
}

void I2C::clearBus(void) {
    gpio_set_direction(sdaPin_, GPIO_MODE_INPUT);
    gpio_set_direction(sclPin_, GPIO_MODE_OUTPUT);

    const uint8_t kClockPulses = 9;
    for (int i = 0; i < kClockPulses; i++) {
        gpio_set_level(sclPin_, 0);
        esp_rom_delay_us(5);  // Clock low period
        gpio_set_level(sclPin_, 1);
        esp_rom_delay_us(5);  // Clock high period
    }

    // Make sure both lines are high to free the bus
    gpio_set_level(sclPin_, 1);
    gpio_set_direction(sdaPin_, GPIO_MODE_INPUT);  // Ensure SDA returns to input mode
}

// -- SPI MOVED TO A SPECIFIC FILE --

/******************************************************************************/
/*                                 Timer                                      */
/******************************************************************************/

Timer::Timer(uint32_t periodMs, int intrPriority, bool autoReload)
    : timerHandle_(nullptr),
      intrPriority_(intrPriority),
      autoReload_(autoReload),
      userCallback_(nullptr),
      callbackArg_(nullptr),
      interruptsEnabled_(true),  // Interrupts enabled by default
      sleepModeEnabled_(false) {
    // Convert period from milliseconds to ticks (1 tick = 1 microsecond)
    alarmCount_ = periodMs * 1000ULL;
}

Timer::Timer() : Timer(250, 1, true) {
}

Timer::Timer(uint32_t periodMs) : Timer(periodMs, 1, true) {
}

Timer::~Timer() {
    deinit();
}

esp_err_t Timer::init(void) {
    gptimer_config_t config = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                               .direction = GPTIMER_COUNT_UP,
                               .resolution_hz = kResolutionHz,
                               .intr_priority = intrPriority_,
                               .flags = {
                                   .intr_shared = false,
                                   .allow_pd = false,
                                   .backup_before_sleep = false,
                               }};

    return gptimer_new_timer(&config, &timerHandle_);
}

esp_err_t Timer::configure(void) {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    gptimer_alarm_config_t alarm_config = {.alarm_count = alarmCount_,
                                           .reload_count = 0,
                                           .flags = {
                                               .auto_reload_on_alarm = autoReload_,
                                           }};

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = Timer::timerISR,
    };

    esp_err_t err = gptimer_register_event_callbacks(timerHandle_, &callbacks, this);
    if (err != ESP_OK) {
        return err;
    }

    return gptimer_set_alarm_action(timerHandle_, &alarm_config);
}

void Timer::setCallback(timer_callback_t callback, void* arg) {
    userCallback_ = callback;
    callbackArg_ = arg;
}

esp_err_t Timer::start(void) {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = gptimer_enable(timerHandle_);
    if (err != ESP_OK) {
        return err;
    }

    return gptimer_start(timerHandle_);
}

esp_err_t Timer::stop(void) {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    return gptimer_stop(timerHandle_);
}

esp_err_t Timer::deinit(void) {
    if (!timerHandle_) {
        return ESP_OK;
    }

    esp_err_t err = gptimer_stop(timerHandle_);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_disable(timerHandle_);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_del_timer(timerHandle_);
    timerHandle_ = nullptr;
    return err;
}

esp_err_t Timer::getCounter(uint64_t* _count) const {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    return gptimer_get_raw_count(timerHandle_, _count);
}

esp_err_t Timer::setCounter(uint64_t count) const {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    return gptimer_set_raw_count(timerHandle_, count);
}

esp_err_t Timer::getCounterUs(uint32_t* us) const {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t ticks = 0;
    esp_err_t err = gptimer_get_raw_count(timerHandle_, &ticks);

    if (err != ESP_OK) {
        return err;
    }

    *us = static_cast<uint32_t>(ticks);
    return ESP_OK;
}

esp_err_t Timer::getCounterMs(uint32_t* ms) const {
    if (!timerHandle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint64_t ticks = 0;
    // Use the raw count API (each tick is 1 microsecond).
    esp_err_t err = gptimer_get_raw_count(timerHandle_, &ticks);
    if (err != ESP_OK) {
        return err;
    }

    // Convert microseconds to milliseconds.
    *ms = static_cast<uint32_t>(ticks / 1000ULL);
    return ESP_OK;
}

esp_err_t Timer::reset(void) const {
    return setCounter(0);
}

esp_err_t Timer::setFrequency(uint32_t frequency) {
    if (frequency == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    // Calculate new alarm count based on desired frequency.
    alarmCount_ = kResolutionHz / frequency;

    if (timerHandle_) {
        return configure();
    }

    return ESP_OK;
}

uint32_t Timer::getFrequency(void) const {
    if (alarmCount_ == 0) {
        return 0;
    }

    return kResolutionHz / alarmCount_;
}

esp_err_t Timer::setPeriodMs(uint32_t periodMs) {
    // Convert period in ms to ticks.
    alarmCount_ = periodMs * 1000ULL;

    if (timerHandle_) {
        return configure();
    }

    return ESP_OK;
}

uint32_t Timer::getPeriodMs(void) const {
    return static_cast<uint32_t>(alarmCount_ / 1000ULL);
}

void Timer::enableInterrupts(void) {
    interruptsEnabled_ = true;
}

void Timer::disableInterrupts(void) {
    interruptsEnabled_ = false;
}

bool IRAM_ATTR Timer::timerISR(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t* eventData,
                               void* userData) {
    Timer* instance = static_cast<Timer*>(userData);
    // Only invoke the user callback if interrupts are enabled.
    if (instance && instance->interruptsEnabled_ && instance->userCallback_) {
        instance->userCallback_(instance->callbackArg_);
    }

    return true;
}

esp_err_t Timer::enableSleepMode(void) {
    esp_err_t err = stop();
    if (err != ESP_OK) {
        return err;
    }

    disableInterrupts();

    err = deinit();
    if (err != ESP_OK) {
        return err;
    }

    sleepModeEnabled_ = true;

    // Reinitialize the timer with sleep configuration enabled:
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = kResolutionHz,
        .intr_priority = intrPriority_,
        .flags = {
            .intr_shared = false,
            .allow_pd = true,  // Enable power domain shutdown in sleep
            .backup_before_sleep = true,
        }};

    err = gptimer_new_timer(&config, &timerHandle_);
    if (err != ESP_OK) {
        return err;
    }

    err = configure();
    if (err != ESP_OK) {
        return err;
    }

    enableInterrupts();
    return ESP_OK;
}

esp_err_t Timer::disableSleepMode(void) {
    esp_err_t err = stop();
    if (err != ESP_OK) {
        return err;
    }

    disableInterrupts();

    err = deinit();
    if (err != ESP_OK) {
        return err;
    }

    sleepModeEnabled_ = false;

    // Reinitialize the timer with sleep configuration disabled:
    // both allow_pd and backup_before_sleep set to false.
    gptimer_config_t config = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                               .direction = GPTIMER_COUNT_UP,
                               .resolution_hz = kResolutionHz,
                               .intr_priority = intrPriority_,
                               .flags = {
                                   .intr_shared = false,
                                   .allow_pd = false,
                                   .backup_before_sleep = false,
                               }};

    err = gptimer_new_timer(&config, &timerHandle_);
    if (err != ESP_OK) {
        return err;
    }

    err = configure();
    if (err != ESP_OK) {
        return err;
    }

    enableInterrupts();

    return ESP_OK;
}
