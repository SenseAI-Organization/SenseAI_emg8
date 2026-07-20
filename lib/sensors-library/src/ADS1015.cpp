
/*******************************************************************************
 * @file ADS1015.cpp
 * @brief Contains the definitions of the ADS1015 class methods.
 *
 * This device is a four channel ADC with 12-bit resolution and a programmable
 * gain amplifier. The device can be configured to use one of four I2C addresses.
 *
 * @version v0.0.1
 * @date 2025-01-16
 * @author daniel@sense-ai.co, Sense AI
 *******************************************************************************/

#include "ADS1015.hpp"

ADS1015::ADS1015(I2C& i2cInstance, uint8_t address)
    : i2c_(i2cInstance), address_(static_cast<ADS1015::ADS111X_Address>(address)) {
}

ADS1015::~ADS1015() {
}

esp_err_t ADS1015::init() {
    // Use i2c_ instead of i2cInstance
    uint8_t data[1] = {0};
    esp_err_t err = i2c_.write(address_, static_cast<uint8_t>(Register::Config), data, 1);
    if (err) {
        return err;
    }
    err = setConfig(0, gain_, sampleFrequency_);
    if (err) {
        return err;
    }
    return ESP_OK;
}

esp_err_t ADS1015::init(uint8_t gain, uint8_t sampleFrequency) {
    gain_ = gain;
    sampleFrequency_ = sampleFrequency;
    return init();
}

esp_err_t ADS1015::measure(void) {
    adcData.channels[0] = readSingleEndedSigned(0);
    adcData.channels[1] = readSingleEndedSigned(1);
    adcData.channels[2] = readSingleEndedSigned(2);
    adcData.channels[3] = readSingleEndedSigned(3);
    return ESP_OK;
}

int16_t ADS1015::getChannel(uint8_t channel) {
    return adcData.channels[channel];
}

esp_err_t ADS1015::measure(uint8_t channel) {
    esp_err_t err = setConfig(channel, gain_, sampleFrequency_);

    if (err) {
        printf("Error setting config: %s\n", esp_err_to_name(err));
        return err;
    }

    bool sampleReady = false;
    while (!sampleReady)  // Convert milliseconds to ticks
    {
        available(&sampleReady);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    err = i2c_.read(address_, static_cast<uint8_t>(Register::Conversion),
                    &rawData.channels[channel * 2], 2);
    return err;
}

uint8_t ADS1015::getID(void) const {
    return address_;
}

esp_err_t ADS1015::getReport(char* _buff) const {
    if (_buff == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(_buff, 128, "ID: %X, ch1: %d, ch2: %d, ch3: %d, ch4: %d", getID(),
             adcData.channels[0], adcData.channels[1], adcData.channels[2],
             adcData.channels[3]);
    printf("%s\n", _buff);

    return ESP_OK;
}

esp_err_t ADS1015::setConfig(uint8_t channel, uint8_t gain, uint8_t sampleFrequency) {
    if (channel > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t config = static_cast<uint16_t>(ConfigOS::Single) |
                      static_cast<uint16_t>(ConfigComparatorQueue::None);

    config = setSampleFrequency(config, sampleFrequency);
    config = setGain(config, gain);
    config |= static_cast<uint16_t>(ConfigMode::Single);
    config = setChannel(config, channel);

    uint8_t configArray[2];
    configArray[0] = (config >> 8) & 0xFF;  // High byte
    configArray[1] = config & 0xFF;         // Low byte

    esp_err_t err =
        i2c_.write(address_, static_cast<uint8_t>(Register::Config), configArray, 2);
    return err;
}

esp_err_t ADS1015::checkForDevice() {
    // Use i2c_ instead of i2cInstance
    uint8_t data[1] = {0};
    for (int addr = ADS111X_ADDR_GND; addr <= ADS111X_ADDR_SCL; ++addr) {
        esp_err_t err = i2c_.write(addr, static_cast<uint8_t>(Register::Config), data, 1);
        if (err) {
            return err;
        }
    }
    // printf("ADS seems to be working\n");
    return ESP_OK;
}

int8_t ADS1015::readSingleEnded(uint8_t channel) {
    esp_err_t err = measure(channel);
    if (err) {
        printf("Error reading data: %s\n", esp_err_to_name(err));
        return 0;
    }

    uint16_t rawValue =
        (rawData.channels[channel * 2] << 8) | rawData.channels[channel * 2 + 1];
    uint16_t result = rawValue >> 4;

    return (result);  // Convert without ambiguity
}

// Returns the sensor channel single-ended input as int16_t (two's complement)
int16_t ADS1015::readSingleEndedSigned(uint8_t channel) {
    esp_err_t err = measure(channel);
    if (err) {
        printf("Error reading data: %s\n", esp_err_to_name(err));
        return 0;
    }

    uint16_t rawValue =
        (rawData.channels[channel * 2] << 8) | rawData.channels[channel * 2 + 1];
    uint16_t result = rawValue >> 4;

    return (convertUnsignedToSigned(result));  // Convert without ambiguity
}

esp_err_t ADS1015::available(bool* sampleReady) {
    uint8_t response[2];
    esp_err_t err =
        i2c_.read(address_, static_cast<uint8_t>(Register::Config), response, 2);

    if (err != ESP_OK) {
        return err;
    }

    uint16_t value = (response[0] << 8) | response[1];
    *sampleReady = bool(((value & static_cast<uint16_t>(ConfigOS::Ready)) >
                         0));  // If the OS bit is 1 : the device is not currently
                               // performing a conversion (i.e. data is available)
    return err;
}

int16_t ADS1015::convertUnsignedToSigned(uint16_t value) {
    if (value & 0x8000) {
        return (int16_t)(value - 0x10000);
    }
    return (int16_t)value;
}

esp_err_t ADS1015::readConfBits(uint8_t* data) {
    // Use i2c_ instead of i2cInstance
    esp_err_t err = i2c_.read(address_, static_cast<uint8_t>(Register::Config), data, 2);
    if (err) {
        return err;
    }
    printf("Data: %d %d\n", data[0], data[1]);
    printf("Data as bits: 0x%02X 0x%02X\n", data[0], data[1]);

    return ESP_OK;
}

uint16_t ADS1015::setSampleFrequency(uint16_t config, uint8_t sampleFrequency) {
    switch (sampleFrequency) {
        case 0:
            return config | static_cast<uint16_t>(ConfigRate::Rate_128Hz);
        case 1:
            return config | static_cast<uint16_t>(ConfigRate::Rate_250Hz);
        case 2:
            return config | static_cast<uint16_t>(ConfigRate::Rate_490Hz);
        case 3:
            return config | static_cast<uint16_t>(ConfigRate::Rate_920Hz);
        case 4:
            return config | static_cast<uint16_t>(ConfigRate::Rate_1600Hz);
        case 5:
            return config | static_cast<uint16_t>(ConfigRate::Rate_2400Hz);
        case 6:
            return config | static_cast<uint16_t>(ConfigRate::Rate_3300Hz);
        default:
            return config;
    }
}

uint16_t ADS1015::setGain(uint16_t config, uint8_t gain) {
    switch (gain) {
        case 0:
            return config | static_cast<uint16_t>(ConfigPGA::TwoThirds);
        case 1:
            return config | static_cast<uint16_t>(ConfigPGA::One);
        case 2:
            return config | static_cast<uint16_t>(ConfigPGA::Two);
        case 3:
            return config | static_cast<uint16_t>(ConfigPGA::Four);
        case 4:
            return config | static_cast<uint16_t>(ConfigPGA::Eight);
        case 5:
            return config | static_cast<uint16_t>(ConfigPGA::Sixteen);
        default:
            return config;
    }
}

uint16_t ADS1015::setChannel(uint16_t config, uint8_t channel) {
    switch (channel) {
        case 0:
            return config | static_cast<uint16_t>(ConfigMux::Single_0);
        case 1:
            return config | static_cast<uint16_t>(ConfigMux::Single_1);
        case 2:
            return config | static_cast<uint16_t>(ConfigMux::Single_2);
        case 3:
            return config | static_cast<uint16_t>(ConfigMux::Single_3);
        default:
            return config;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Continuous / ALERT-RDY mode implementation
// ═══════════════════════════════════════════════════════════════════════════════

ADS1015::ConfigMux ADS1015::channelToMux(uint8_t channel) {
    switch (channel) {
        case 0:
            return ConfigMux::Single_0;
        case 1:
            return ConfigMux::Single_1;
        case 2:
            return ConfigMux::Single_2;
        case 3:
            return ConfigMux::Single_3;
        default:
            return ConfigMux::Single_0;
    }
}

uint16_t ADS1015::buildContinuousConfig(ConfigMux mux, ConfigRate rate, ConfigPGA gain) {
    uint16_t config = 0;
    config |= static_cast<uint16_t>(mux);
    config |= static_cast<uint16_t>(gain);
    config |= static_cast<uint16_t>(ConfigMode::Continuous);
    config |= static_cast<uint16_t>(rate);
    // Comparator: traditional, active-low, non-latching, assert after 1 conversion
    // This turns ALERT/RDY into a data-ready signal
    config |= static_cast<uint16_t>(ConfigComparatorMode::Traditional);
    config |= static_cast<uint16_t>(ConfigComparatorPolarity::ActiveLow);
    config |= static_cast<uint16_t>(ConfigComparatorLatching::NonLatching);
    config |= static_cast<uint16_t>(ConfigComparatorQueue::OneConversion);
    return config;
}

esp_err_t ADS1015::writeConfig(uint16_t config) {
    return writeRegister(Register::Config, config);
}

esp_err_t ADS1015::writeRegister(Register reg, uint16_t value) {
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    return i2c_.write(address_, static_cast<uint8_t>(reg), buf, 2);
}

int16_t ADS1015::readConversionResult() {
    uint8_t buf[2] = {0, 0};
    esp_err_t err =
        i2c_.read(address_, static_cast<uint8_t>(Register::Conversion), buf, 2);
    if (err != ESP_OK) {
        return 0;
    }
    // ADS1015: 12-bit result in the upper 12 bits of the 16-bit register
    int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
    return raw >> 4;  // Sign-extending arithmetic shift
}

esp_err_t ADS1015::configureAlertPin(gpio_num_t alertPin, bool activeLow) {
    alertPin_ = alertPin;

    // Configure ALERT/RDY as data-ready: ThreshLo = 0x0000, ThreshHi = 0x8000
    // Per ADS1015 datasheet §9.3.6: this makes ALERT assert after every conversion
    esp_err_t err = writeRegister(Register::ThreshLow, 0x0000);
    if (err != ESP_OK) return err;
    err = writeRegister(Register::ThreshHigh, 0x8000);
    if (err != ESP_OK) return err;

    // Configure GPIO as input with interrupt
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << alertPin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = activeLow ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = activeLow ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = activeLow ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
    err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

void IRAM_ATTR ADS1015::alertISR(void* arg) {
    ADS1015* self = static_cast<ADS1015*>(arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(self->drdySemaphore_, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void ADS1015::continuousTask(void* pvParam) {
    ADS1015* self = static_cast<ADS1015*>(pvParam);

    while (self->continuousRunning_) {
        // Wait for DRDY interrupt (timeout protects against missed edges)
        if (xSemaphoreTake(self->drdySemaphore_, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (!self->continuousRunning_) break;

            // Read the conversion result for the current channel
            uint8_t ch = self->activeChannels_[self->currentMuxIndex_];
            int16_t value = self->readConversionResult();
            self->latestReading_[ch] = value;

            // Also update the legacy adcData struct
            self->adcData.channels[ch] = (uint16_t)value;

            // Notify user callback if registered
            if (self->convCallback_) {
                self->convCallback_(ch, value, self->convCallbackArg_);
            }

            // Advance to next channel in the round-robin
            self->currentMuxIndex_++;
            if (self->currentMuxIndex_ >= self->numActiveChannels_) {
                self->currentMuxIndex_ = 0;
            }

            // Switch MUX to the next channel (still in continuous mode,
            // so the ADC immediately starts a new conversion)
            uint8_t nextCh = self->activeChannels_[self->currentMuxIndex_];
            ConfigMux nextMux = channelToMux(nextCh);
            uint16_t config = self->buildContinuousConfig(nextMux, self->continuousRate_,
                                                          self->continuousPGA_);
            self->writeConfig(config);
        }
    }

    // Task exits cleanly
    self->continuousTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t ADS1015::startContinuous(const uint8_t* channels, uint8_t numChannels,
                                   ConfigRate rate, ConfigPGA gain) {
    if (numChannels == 0 || numChannels > kMaxActiveChannels) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < numChannels; i++) {
        if (channels[i] > 3) return ESP_ERR_INVALID_ARG;
    }
    if (alertPin_ == GPIO_NUM_NC) {
        printf("ADS1015: ALERT pin not configured. Call configureAlertPin() first.\n");
        return ESP_ERR_INVALID_STATE;
    }

    // Stop any existing continuous session
    if (continuousRunning_) {
        stopContinuous();
    }

    // Store channel list and settings
    numActiveChannels_ = numChannels;
    memcpy(activeChannels_, channels, numChannels);
    currentMuxIndex_ = 0;
    continuousRate_ = rate;
    continuousPGA_ = gain;
    memset((void*)latestReading_, 0, sizeof(latestReading_));

    // Create the binary semaphore
    if (drdySemaphore_ == nullptr) {
        drdySemaphore_ = xSemaphoreCreateBinary();
    }

    // Install GPIO ISR service (safe to call multiple times if already installed)
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    esp_err_t err = gpio_isr_handler_add(alertPin_, alertISR, this);
    if (err != ESP_OK) return err;

    // Mark running before starting the task
    continuousRunning_ = true;

    // Create the handler task (high priority for low latency)
    BaseType_t ret = xTaskCreatePinnedToCore(
        continuousTask, "ads_cont",
        3072,  // Stack size (bytes)
        this,
        configMAX_PRIORITIES - 1,  // High priority
        &continuousTaskHandle_,
        1  // Pin to core 1 so it doesn't compete with WiFi/BT on core 0
    );
    if (ret != pdPASS) {
        continuousRunning_ = false;
        gpio_isr_handler_remove(alertPin_);
        return ESP_ERR_NO_MEM;
    }

    // Kick off the first conversion: write config for channel[0] in continuous mode
    ConfigMux firstMux = channelToMux(activeChannels_[0]);
    uint16_t config = buildContinuousConfig(firstMux, rate, gain);
    err = writeConfig(config);
    if (err != ESP_OK) {
        stopContinuous();
        return err;
    }

    return ESP_OK;
}

esp_err_t ADS1015::stopContinuous() {
    continuousRunning_ = false;

    // Remove the ISR handler
    if (alertPin_ != GPIO_NUM_NC) {
        gpio_isr_handler_remove(alertPin_);
    }

    // Wait for task to exit
    if (continuousTaskHandle_ != nullptr) {
        // Give the semaphore to unblock the task if it's waiting
        if (drdySemaphore_) {
            xSemaphoreGive(drdySemaphore_);
        }
        // Brief wait for task to self-delete
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Put the ADS1015 into single-shot / power-down mode
    uint16_t config = static_cast<uint16_t>(ConfigOS::Single) |
                      static_cast<uint16_t>(ConfigMode::Single) |
                      static_cast<uint16_t>(ConfigMux::Single_0) |
                      static_cast<uint16_t>(ConfigPGA::One) |
                      static_cast<uint16_t>(ConfigRate::Rate_1600Hz) |
                      static_cast<uint16_t>(ConfigComparatorQueue::None);
    return writeConfig(config);
}

bool ADS1015::isContinuousRunning() const {
    return continuousRunning_;
}

int16_t ADS1015::getLatestReading(uint8_t channel) const {
    if (channel >= kMaxChannels) return 0;
    return latestReading_[channel];
}

void ADS1015::onConversion(ConversionCallback cb, void* arg) {
    convCallback_ = cb;
    convCallbackArg_ = arg;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Mixed-rate continuous mode
// ═══════════════════════════════════════════════════════════════════════════════

uint8_t ADS1015::nextMixedChannel() {
    // Check if any slow channel is due for a sample. Each slow channel keeps
    // its own next-due cycle count, advanced on injection — a plain
    // (fastCycleCount_ % divider) test would keep matching while slow
    // channels are injected (the count only advances on fast cycles) and
    // lock the round-robin into slow channels forever.
    for (uint8_t i = 0; i < numSlowChannels_; i++) {
        uint8_t idx = (slowIndex_ + i) % numSlowChannels_;
        if (fastCycleCount_ >= slowNextDue_[idx]) {
            slowNextDue_[idx] += slowDividers_[idx];
            slowIndex_ = (idx + 1) % numSlowChannels_;
            return slowChannels_[idx];
        }
    }

    // Default: next fast channel in round-robin
    uint8_t ch = fastChannels_[fastIndex_];
    fastIndex_++;
    if (fastIndex_ >= numFastChannels_) {
        fastIndex_ = 0;
        fastCycleCount_++;
    }
    return ch;
}

void ADS1015::mixedContinuousTask(void* pvParam) {
    ADS1015* self = static_cast<ADS1015*>(pvParam);

    while (self->continuousRunning_) {
        if (xSemaphoreTake(self->drdySemaphore_, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (!self->continuousRunning_) break;

            // Read the result of the conversion that just finished
            uint8_t ch = self->activeChannels_[0];  // current channel being read
            int16_t value = self->readConversionResult();
            self->latestReading_[ch] = value;
            self->adcData.channels[ch] = (uint16_t)value;

            if (self->convCallback_) {
                self->convCallback_(ch, value, self->convCallbackArg_);
            }

            // Decide next channel and switch MUX
            uint8_t nextCh = self->nextMixedChannel();
            self->activeChannels_[0] = nextCh;  // track what's currently converting

            // Find the gain for this channel
            ConfigPGA nextGain = self->continuousPGA_;
            for (uint8_t i = 0; i < self->numChannelConfigs_; i++) {
                if (self->channelConfigs_[i].channel == nextCh) {
                    nextGain = self->channelConfigs_[i].gain;
                    break;
                }
            }

            ConfigMux nextMux = channelToMux(nextCh);
            uint16_t config =
                self->buildContinuousConfig(nextMux, self->continuousRate_, nextGain);
            self->writeConfig(config);
        }
    }

    self->continuousTaskHandle_ = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t ADS1015::startMixedContinuous(const ChannelConfig* configs, uint8_t numConfigs,
                                        ConfigRate rate) {
    if (numConfigs == 0 || numConfigs > kMaxActiveChannels) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < numConfigs; i++) {
        if (configs[i].channel > 3 || configs[i].divider == 0) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (alertPin_ == GPIO_NUM_NC) {
        printf("ADS1015: ALERT pin not configured. Call configureAlertPin() first.\n");
        return ESP_ERR_INVALID_STATE;
    }

    if (continuousRunning_) {
        stopContinuous();
    }

    // Store configs
    numChannelConfigs_ = numConfigs;
    memcpy(channelConfigs_, configs, numConfigs * sizeof(ChannelConfig));
    continuousRate_ = rate;
    mixedMode_ = true;

    // Separate fast (divider==1) from slow (divider>1) channels
    numFastChannels_ = 0;
    numSlowChannels_ = 0;
    for (uint8_t i = 0; i < numConfigs; i++) {
        if (configs[i].divider == 1) {
            fastChannels_[numFastChannels_++] = configs[i].channel;
        } else {
            slowChannels_[numSlowChannels_] = configs[i].channel;
            slowDividers_[numSlowChannels_] = configs[i].divider;
            slowNextDue_[numSlowChannels_] = configs[i].divider;
            numSlowChannels_++;
        }
    }

    if (numFastChannels_ == 0) {
        printf("ADS1015: Mixed mode needs at least one fast channel (divider=1).\n");
        return ESP_ERR_INVALID_ARG;
    }

    // Reset scheduler state
    fastCycleCount_ = 0;
    fastIndex_ = 0;
    slowIndex_ = 0;
    memset((void*)latestReading_, 0, sizeof(latestReading_));

    // Create semaphore
    if (drdySemaphore_ == nullptr) {
        drdySemaphore_ = xSemaphoreCreateBinary();
    }

    // Install ISR
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    esp_err_t err = gpio_isr_handler_add(alertPin_, alertISR, this);
    if (err != ESP_OK) return err;

    continuousRunning_ = true;

    // First channel to convert
    uint8_t firstCh = fastChannels_[0];
    activeChannels_[0] = firstCh;

    // Advance the scheduler past the first element
    fastIndex_ = 1;
    if (fastIndex_ >= numFastChannels_) {
        fastIndex_ = 0;
        fastCycleCount_++;
    }

    // Create task
    BaseType_t ret =
        xTaskCreatePinnedToCore(mixedContinuousTask, "ads_mixed", 3072, this,
                                configMAX_PRIORITIES - 1, &continuousTaskHandle_, 1);
    if (ret != pdPASS) {
        continuousRunning_ = false;
        gpio_isr_handler_remove(alertPin_);
        return ESP_ERR_NO_MEM;
    }

    // Find gain for first channel
    ConfigPGA firstGain = ConfigPGA::One;
    for (uint8_t i = 0; i < numConfigs; i++) {
        if (configs[i].channel == firstCh) {
            firstGain = configs[i].gain;
            break;
        }
    }

    // Kick off first conversion
    ConfigMux firstMux = channelToMux(firstCh);
    uint16_t config = buildContinuousConfig(firstMux, rate, firstGain);
    err = writeConfig(config);
    if (err != ESP_OK) {
        stopContinuous();
        return err;
    }

    return ESP_OK;
}

esp_err_t ADS1015::startMixedContinuousExternal(const ChannelConfig* configs,
                                                uint8_t numConfigs, ConfigRate rate) {
    if (numConfigs == 0 || numConfigs > kMaxActiveChannels) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint8_t i = 0; i < numConfigs; i++) {
        if (configs[i].channel > 3 || configs[i].divider == 0) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (alertPin_ == GPIO_NUM_NC) {
        printf("ADS1015: ALERT pin not configured. Call configureAlertPin() first.\n");
        return ESP_ERR_INVALID_STATE;
    }

    if (continuousRunning_) {
        stopContinuous();
    }

    // Store configs
    numChannelConfigs_ = numConfigs;
    memcpy(channelConfigs_, configs, numConfigs * sizeof(ChannelConfig));
    continuousRate_ = rate;
    mixedMode_ = true;

    // Separate fast (divider==1) from slow (divider>1) channels
    numFastChannels_ = 0;
    numSlowChannels_ = 0;
    for (uint8_t i = 0; i < numConfigs; i++) {
        if (configs[i].divider == 1) {
            fastChannels_[numFastChannels_++] = configs[i].channel;
        } else {
            slowChannels_[numSlowChannels_] = configs[i].channel;
            slowDividers_[numSlowChannels_] = configs[i].divider;
            slowNextDue_[numSlowChannels_] = configs[i].divider;
            numSlowChannels_++;
        }
    }

    if (numFastChannels_ == 0) {
        printf("ADS1015: Mixed mode needs at least one fast channel (divider=1).\n");
        return ESP_ERR_INVALID_ARG;
    }

    // Reset scheduler state
    fastCycleCount_ = 0;
    fastIndex_ = 0;
    slowIndex_ = 0;
    memset((void*)latestReading_, 0, sizeof(latestReading_));

    // Create semaphore
    if (drdySemaphore_ == nullptr) {
        drdySemaphore_ = xSemaphoreCreateBinary();
    }

    // Install ISR
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    esp_err_t err = gpio_isr_handler_add(alertPin_, alertISR, this);
    if (err != ESP_OK) return err;

    continuousRunning_ = true;
    continuousTaskHandle_ = nullptr;  // No internal task

    // First channel to convert
    uint8_t firstCh = fastChannels_[0];
    activeChannels_[0] = firstCh;

    // Advance the scheduler past the first element
    fastIndex_ = 1;
    if (fastIndex_ >= numFastChannels_) {
        fastIndex_ = 0;
        fastCycleCount_++;
    }

    // Find gain for first channel
    ConfigPGA firstGain = ConfigPGA::One;
    for (uint8_t i = 0; i < numConfigs; i++) {
        if (configs[i].channel == firstCh) {
            firstGain = configs[i].gain;
            break;
        }
    }

    // Kick off first conversion
    ConfigMux firstMux = channelToMux(firstCh);
    uint16_t config = buildContinuousConfig(firstMux, rate, firstGain);
    err = writeConfig(config);
    if (err != ESP_OK) {
        stopContinuous();
        return err;
    }

    return ESP_OK;
}

bool ADS1015::serviceConversion() {
    if (!continuousRunning_ || drdySemaphore_ == nullptr) return false;

    if (xSemaphoreTake(drdySemaphore_, 0) != pdTRUE) return false;
    if (!continuousRunning_) return false;

    // Read the result of the conversion that just finished
    uint8_t ch = activeChannels_[0];
    int16_t value = readConversionResult();
    latestReading_[ch] = value;
    adcData.channels[ch] = (uint16_t)value;

    if (convCallback_) {
        convCallback_(ch, value, convCallbackArg_);
    }

    // Decide next channel and switch MUX
    uint8_t nextCh = nextMixedChannel();
    activeChannels_[0] = nextCh;

    // Find the gain for this channel
    ConfigPGA nextGain = continuousPGA_;
    for (uint8_t i = 0; i < numChannelConfigs_; i++) {
        if (channelConfigs_[i].channel == nextCh) {
            nextGain = channelConfigs_[i].gain;
            break;
        }
    }

    ConfigMux nextMux = channelToMux(nextCh);
    uint16_t config = buildContinuousConfig(nextMux, continuousRate_, nextGain);
    writeConfig(config);

    return true;
}

float ADS1015::getEffectiveSampleRate(uint8_t channel) const {
    if (!continuousRunning_ || !mixedMode_) return 0.0f;

    // Lookup the config for this channel
    const ChannelConfig* cfg = nullptr;
    for (uint8_t i = 0; i < numChannelConfigs_; i++) {
        if (channelConfigs_[i].channel == channel) {
            cfg = &channelConfigs_[i];
            break;
        }
    }
    if (!cfg) return 0.0f;

    // Nominal hardware SPS
    float baseSps = 0;
    switch (continuousRate_) {
        case ConfigRate::Rate_128Hz:
            baseSps = 128.0f;
            break;
        case ConfigRate::Rate_250Hz:
            baseSps = 250.0f;
            break;
        case ConfigRate::Rate_490Hz:
            baseSps = 490.0f;
            break;
        case ConfigRate::Rate_920Hz:
            baseSps = 920.0f;
            break;
        case ConfigRate::Rate_1600Hz:
            baseSps = 1600.0f;
            break;
        case ConfigRate::Rate_2400Hz:
            baseSps = 2400.0f;
            break;
        case ConfigRate::Rate_3300Hz:
            baseSps = 3300.0f;
            break;
    }

    // Total conversion slots per fast cycle = numFastChannels + injected slow samples
    // Slow channels inject ~1/divider samples per fast cycle
    float slotsPerCycle = (float)numFastChannels_;
    for (uint8_t i = 0; i < numSlowChannels_; i++) {
        slotsPerCycle += 1.0f / slowDividers_[i];
    }

    float effectiveRate = baseSps / slotsPerCycle;

    if (cfg->divider > 1) {
        // Slow channel: sampled once per divider fast-cycles
        effectiveRate = effectiveRate / cfg->divider;
    }

    return effectiveRate;
}
