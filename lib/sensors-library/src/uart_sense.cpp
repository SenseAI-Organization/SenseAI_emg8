/*******************************************************************************
 * @file uart_sense.hpp
 * @brief Contains methods to handle UART communication for ESP32 SoCs as part
 * of the Sense Ecosystem.
 *
 * @version 0.3.1
 * @date 2025-06-27
 * @author Mateo R.B. (mateor@sense-ai.co)

 *******************************************************************************
 *******************************************************************************/

#include "driver/gpio.h"
#include "uart_sense.hpp"

UART::UART(int baudRate, uint16_t maxBufferSize, bool allowEvents)
    : UART::UART(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, baudRate, maxBufferSize, kDefaultQueueSize,
                 allowEvents) {};

UART::UART(uart_port_t port, int baudRate, uint16_t maxBufferSize, bool allowEvents)
    : UART::UART(port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, baudRate, maxBufferSize, kDefaultQueueSize,
                 allowEvents) {};

UART::UART(uart_port_t port, int TxPin, int RxPin, int baudRate, uint16_t maxBufferSize,
           bool allowEvents)
    : UART::UART(port, TxPin, RxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, baudRate,
                 maxBufferSize, kDefaultQueueSize, allowEvents) {};

// Base constructor
UART::UART(uart_port_t port, int txPin, int rxPin, int rtsPin, int ctsPin, int baudRate,
           uint16_t maxBufferSize, uint16_t queueSize, bool allowEvents,
           bool rxInternalPullup)
    : port_(port),
      txPin_(txPin),
      rxPin_(rxPin),
      rtsPin_(rtsPin),
      ctsPin_(ctsPin),
      baudRate_(baudRate),
      rxInternalPullup_(rxInternalPullup),
      maxBufferSize_(maxBufferSize),
      queueSize_(queueSize),
      allowEvents_(allowEvents) {
    // Initialize default values
    rxThresh_ = static_cast<uint8_t>(UART_HW_FIFO_LEN(port_) - 1);
    dataLen_ = 0;
    dataAvailable_ = false;
    isHW_flowControlSet = false;
    QueueHandle_ = nullptr;
    echoEnable_ = false;  // Echo is disabled by default
    stackSize_ = 4096;    // Default stack size for the event handler task
    rxBuffer_ = std::string();
};

UART::~UART() {
    deinit();
}

esp_err_t UART::init() {
    rxThresh_ = static_cast<uint8_t> UART_HW_FIFO_LEN(port_) - 1;
    int intr_alloc_flag = 0;

    // Default config, it can be overwritting later
    uart_config_t uartConfig = {
        .baud_rate = baudRate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // SW flow control by default
        .rx_flow_ctrl_thresh =
            rxThresh_,  // max FIFO slots are 128, threshold must be < 128
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {.allow_pd = 0, .backup_before_sleep = 0}};

    esp_err_t err = uart_param_config(port_, &uartConfig);
    if (err) {
        return err;
    }

    if (allowEvents_) {
        // Create the eventHandler task name with the port number in a string
        taskName_ = "Event handler task UART" + std::to_string(port_);
        err = uart_driver_install(port_, maxBufferSize_, maxBufferSize_, queueSize_,
                                  &QueueHandle_, intr_alloc_flag);
        xTaskCreate(UART::eventHandler, taskName_.c_str(), stackSize_, this, 12,
                    &taskHandle_);
    } else {
        err = uart_driver_install(port_, maxBufferSize_, 0, 0, nullptr, intr_alloc_flag);
    }

    if (err) {
        return err;
    }

    err = uart_set_pin(port_, txPin_, rxPin_, rtsPin_, ctsPin_);
    if (err) {
        return err;
    }

    // Configure RX pull-up if requested and RX pin is valid
    if (rxPin_ != UART_PIN_NO_CHANGE && rxInternalPullup_) {
        err = gpio_set_pull_mode(static_cast<gpio_num_t>(rxPin_), GPIO_PULLUP_ONLY);
        if (err) {
            return err;
        }
    }

    return ESP_OK;
}

void UART::deinit() {
    rxBuffer_.clear();
    if (QueueHandle_) {
        vQueueDelete(QueueHandle_);
        QueueHandle_ = nullptr;
    }

    if (allowEvents_) {
        if (taskHandle_ != nullptr) {
            vTaskDelete(taskHandle_);
            taskHandle_ = nullptr;
        }
    }

    uart_driver_delete(port_);
    dataAvailable_ = false;
    echoEnable_ = false;
    isHW_flowControlSet = false;
    dataLen_ = 0;
}

esp_err_t UART::setBaudRate(int newBaudrate) {
    baudRate_ = newBaudrate;
    return uart_set_baudrate(port_, baudRate_);
}

esp_err_t UART::setDataBits(WordLength newDataBits) {
    return uart_set_word_length(port_, static_cast<uart_word_length_t>(newDataBits));
}

esp_err_t UART::setParity(ParityMode newParity) {
    return uart_set_parity(port_, static_cast<uart_parity_t>(newParity));
}

esp_err_t UART::setStopBits(StopBits newStopBits) {
    return uart_set_stop_bits(port_, static_cast<uart_stop_bits_t>(newStopBits));
}

esp_err_t UART::setUartMode(UartMode newMode) {
    // we need rs485 and irda devices to test this mode...
    return ESP_FAIL;
}

esp_err_t UART::setHW_flowCtrlMode(FlowControl newFlowCtrlMode) {
    esp_err_t err;

    if (newFlowCtrlMode == FlowControl::kSoftwareCtrl) {
        err = uart_set_sw_flow_ctrl(port_, true, 0, rxThresh_);
        if (!err) {
            isHW_flowControlSet = false;
        }
    } else {
        err = uart_set_hw_flow_ctrl(
            port_, static_cast<uart_hw_flowcontrol_t>(newFlowCtrlMode), rxThresh_);
        if (!err) {
            isHW_flowControlSet = true;
        }
    }

    return err;
}

esp_err_t UART::getBaudRate(int* pBaudRate) {
    return uart_get_baudrate(port_, (uint32_t*)pBaudRate);
}

esp_err_t UART::getDataBits(WordLength* pWordLen) {
    return uart_get_word_length(port_, (uart_word_length_t*)pWordLen);
}

esp_err_t UART::getParity(ParityMode* pParity) {
    return uart_get_parity(port_, (uart_parity_t*)pParity);
}

esp_err_t UART::getStopBits(StopBits* pStopBits) {
    return uart_get_stop_bits(port_, (uart_stop_bits_t*)pStopBits);
}

esp_err_t UART::getFlowCtrlMode(FlowControl* pFlowCtrlMode) {
    return uart_get_hw_flow_ctrl(port_, (uart_hw_flowcontrol_t*)pFlowCtrlMode);
}

esp_err_t UART::getUartMode(UartMode* pUartMode) {
    return ESP_FAIL;
}  // NOT IN USE YET!!!

int UART::getDataLen(void) {
    if (allowEvents_) {
        // Only eventHandler should update dataAvailable_ in this mode
        return dataLen_;
    } else {
        size_t buffered_len = 0;
        esp_err_t err = uart_get_buffered_data_len(port_, &buffered_len);
        if (err != ESP_OK) {
            return -1;  // Error in getting data length
        }

        // Only set dataAvailable_ if there is new data and it was previously
        // false
        if (buffered_len > 0 && !dataAvailable_) {
            dataAvailable_ = true;
        }

        return static_cast<int>(buffered_len);
    }
}

esp_err_t UART::read(std::string& bufferToStore, size_t msgSize) {
    // Use getDataLen() to check if there is data available
    if (!allowEvents_) {
        int len = getDataLen();
        if (len <= 0) {
            dataAvailable_ = false;        // Reset data available flag
            return ESP_ERR_INVALID_STATE;  // No data available to read
        }
    }

    if (msgSize > maxBufferSize_) {
        return ESP_ERR_INVALID_SIZE;  // Ensure the requested size is within
                                      // limits
    }

    if (msgSize < 1) {
        msgSize = maxBufferSize_;  // Use the available data length if msgSize is 0
    }

    // Clear rxBuffer_ before reading new data
    rxBuffer_.clear();

    char* auxBuffer = new char[msgSize + 1];  // Allocate buffer for reading
    dataLen_ = uart_read_bytes(port_, auxBuffer, msgSize, pdMS_TO_TICKS(100));

    if (dataLen_ == -1) {
        delete[] auxBuffer;
        return ESP_FAIL;
    }

    if (dataLen_ == 0) {
        delete[] auxBuffer;
        return ESP_ERR_INVALID_RESPONSE;  // No data received
    }

    rxBuffer_ = std::string(auxBuffer, dataLen_);  // Save a copy of the data
    bufferToStore = rxBuffer_;  // Store the data in the provided buffer

    if (echoEnable_) {
        echo();  // Echo the received data if echo is enabled
    }

    delete[] auxBuffer;
    dataAvailable_ = false;  // Reset data available flag ONLY here
    return ESP_OK;
}

void UART::getData(std::string& _bufferToStore) {
    _bufferToStore = rxBuffer_;
}

esp_err_t UART::write(const std::string& bufferToWrite) {
    if (uart_write_bytes(port_, bufferToWrite.data(), bufferToWrite.size()) == -1) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void UART::eventHandler(void* pvParameters) {
    UART* uart = static_cast<UART*>(pvParameters);
    uart_event_t s_event;
    size_t buffered_len = 0;

    while (1) {
        if (xQueueReceive(uart->QueueHandle_, (void*)&s_event, pdMS_TO_TICKS(100))) {
            switch (s_event.type) {
                case UART_DATA:
                    if (s_event.size < 1) {
                        break;
                    }

                    uart_get_buffered_data_len(uart->port_, &buffered_len);
                    if (buffered_len < 1) {
                        break;
                    }

                    uart->dataLen_ = static_cast<int16_t>(buffered_len);
                    uart->dataAvailable_ = true;
                    break;
                case UART_BREAK:  // when data transfer was interrupted
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(uart->port_);
                    xQueueReset(uart->QueueHandle_);
                    break;
                case UART_FIFO_OVF:
                    break;
                case UART_FRAME_ERR:
                    break;
                case UART_PARITY_ERR:
                    break;
                case UART_DATA_BREAK:
                    break;
                case UART_PATTERN_DET:  // useful to command detection
                    break;
                default:
                    break;
            }
        }
    }
}

esp_err_t UART::echo() {
    esp_err_t err = write(rxBuffer_);
    isClearCmd();  // Check if the "clear" command was received
    return err;
}

void UART::enableEcho(void) {
    echoEnable_ = true;
}

void UART::disableEcho(void) {
    echoEnable_ = false;
}

bool UART::isDataAvailable(void) {
    // Only eventHandler or getDataLen (when allowEvents_ == false) should set
    // dataAvailable_
    return dataAvailable_;
}

void UART::clearTerminal(void) {
    UART::write(std::string(&clearingChar_, 1));
}

void UART::isClearCmd(void) {
    if (rxBuffer_.compare(0, 5, "clear", 0, 5) == 0) {
        clearTerminal();
    }
}

esp_err_t UART::clearRxBuffer(void) {
    return uart_flush_input(port_);
}

esp_err_t UART::setRxPinInternalPullup(bool enable) {
    if (rxPin_ == UART_PIN_NO_CHANGE) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_pull_mode_t mode = enable ? GPIO_PULLUP_ONLY : GPIO_FLOATING;
    esp_err_t err = gpio_set_pull_mode(static_cast<gpio_num_t>(rxPin_), mode);

    if (!err) {
        rxInternalPullup_ = enable;
    }

    return err;
}

bool UART::getRxPinInternalPullup(void) const {
    return rxInternalPullup_;
}
