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

#pragma once

#include <string>

#include "driver/uart.h"

/******************************************************************************/
/*                                 UART                                       */
/******************************************************************************/

/**
 * @class UART
 *
 * @brief A class to manage the UART peripheral
 */
class UART {
public:
    /**
     * @enum WordLength
     * @brief Enumeration for tx/rx data length
     */
    enum class WordLength : uint8_t {
        kDataBits_5 = UART_DATA_5_BITS,
        kDataBits_6 = UART_DATA_6_BITS,
        kDataBits_7 = UART_DATA_7_BITS,
        kDataBits_8 = UART_DATA_8_BITS
    };

    /**
     * @enum ParityMode
     * @brief Enumeration for uart parity
     */
    enum class ParityMode : uint8_t {
        kParityDisable = UART_PARITY_DISABLE,
        kParityEven = UART_PARITY_EVEN,
        kParityOdd = UART_PARITY_ODD
    };

    /**
     * @enum StopBits
     * @brief Enumeration for uart stop bits
     */
    enum class StopBits : uint8_t {
        kStopBits_1 = UART_STOP_BITS_1,
        kStopBits_1_5 = UART_STOP_BITS_1_5,
        kStopBits_2 = UART_STOP_BITS_2
    };

    /**
     * @enum UartMode
     * @brief Enumeration for uart mode
     * @note Default mode is known as RS232, other uart protocols are RS485 and
     * IrDA (Infrared Data Association)
     */
    enum class UartMode : uint8_t {
        kUartDefault = UART_MODE_UART,
        kRs485_halfDuplex = UART_MODE_RS485_HALF_DUPLEX,
        kIrda = UART_MODE_IRDA,
        kRs485_collisionDetect = UART_MODE_RS485_COLLISION_DETECT,
        kRs485_appCtrl = UART_MODE_RS485_APP_CTRL
    };

    /**
     * @enum FlowControl
     * @brief Enumeration for uart flow control selection
     * @note There are more modes but for now those are the only available by
     * our hardware
     */
    enum class FlowControl : uint8_t {
        kSoftwareCtrl = UART_HW_FLOWCTRL_DISABLE,
        kHardwareCtrl_RTS = UART_HW_FLOWCTRL_RTS,
        kHardwareCtrl_CTS = UART_HW_FLOWCTRL_CTS,
        kHardwareCtrl_CTS_RTS = UART_HW_FLOWCTRL_CTS_RTS
    };

    /**
     * @brief Construct a new UART object with full configuration control
     *
     * @param port UART port number (UART_NUM_0, UART_NUM_1, etc.)
     * @param txPin GPIO pin number for TX (use UART_PIN_NO_CHANGE for default)
     * @param rxPin GPIO pin number for RX (use UART_PIN_NO_CHANGE for default)
     * @param rtsPin GPIO pin number for RTS (use UART_PIN_NO_CHANGE to disable)
     * @param ctsPin GPIO pin number for CTS (use UART_PIN_NO_CHANGE to disable)
     * @param baudRate UART baud rate (e.g., 9600, 115200)
     * @param maxBufferSize Maximum size for RX buffer in bytes (default: 1024)
     * @param queueSize Maximum number of RX data chunks in event queue (default: 10)
     * @param allowEvents Enable UART event handler task (default: true)
     * @param rxInternalPullup Enable internal pull-up resistor on RX pin (default:
     * false)
     */
    UART(uart_port_t port, int txPin, int rxPin, int rtsPin, int ctsPin, int baudRate,
         uint16_t maxBufferSize, uint16_t queueSize, bool allowEvents,
         bool rxInternalPullup = false);

    /**
     * @brief Construct a new UART object with default port and pin configuration
     *
     * @param baudRate UART baud rate (e.g., 9600, 115200)
     * @param maxBufferSize Maximum size for RX buffer in bytes (default: 1024)
     * @param allowEvents Enable UART event handler task (default: true)
     *
     * @note Uses default port (UART_NUM_0) with default ESP32 pins:
     *       - TX Pin: UART_PIN_NO_CHANGE (uses default)
     *       - RX Pin: UART_PIN_NO_CHANGE (uses default)
     *       - RTS Pin: Not used
     *       - CTS Pin: Not used
     */
    UART(int baudRate, uint16_t maxBufferSize = 1024, bool allowEvents = true);

    /**
     * @brief Construct a new UART object with specific port and default pin configuration
     *
     * @param port UART port number (UART_NUM_0, UART_NUM_1, etc.)
     * @param baudRate UART baud rate (e.g., 9600, 115200)
     * @param maxBufferSize Maximum size for RX buffer in bytes (default: 1024)
     * @param allowEvents Enable UART event handler task (default: true)
     *
     * @note Uses default pins for the specified port (UART_PIN_NO_CHANGE)
     */
    UART(uart_port_t port, int baudRate, uint16_t maxBufferSize = 1024,
         bool allowEvents = true);

    /**
     * @brief Construct a new UART object with specific port and TX/RX pins
     *
     * @param port UART port number (UART_NUM_0, UART_NUM_1, etc.)
     * @param txPin GPIO pin number for TX
     * @param rxPin GPIO pin number for RX
     * @param baudRate UART baud rate (e.g., 9600, 115200)
     * @param maxBufferSize Maximum size for RX buffer in bytes (default: 1024)
     * @param allowEvents Enable UART event handler task (default: true)
     *
     * @note RTS and CTS pins are not configured (flow control disabled)
     */
    UART(uart_port_t port, int txPin, int rxPin, int baudRate,
         uint16_t maxBufferSize = 1024, bool allowEvents = true);

    /**
     * @brief Destructor for the UART object
     */
    ~UART();

    /**
     * @brief Initialize the UART peripheral with the parameters given in the constructor
     *
     * Configures the UART hardware, installs the driver, sets pins, and creates
     * the event handler task if enabled.
     *
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error or initialization failure
     */
    esp_err_t init(void);

    /**
     * @brief Deinitialize and release all UART peripheral resources
     *
     * Clears buffers, deletes event handler task and queue, and uninstalls
     * the UART driver. Called automatically by destructor.
     */
    void deinit(void);

    // Runtime configuration functions
    /**
     * @brief Set the baud rate for UART communication
     *
     * @param newBaudrate New baud rate value (e.g., 9600, 115200)
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t setBaudRate(int newBaudrate);

    /**
     * @brief Set the data word length for UART communication
     *
     * @param newDataBits Data bits configuration (5, 6, 7, or 8 bits)
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t setDataBits(WordLength newDataBits);

    /**
     * @brief Set the parity mode for error detection
     *
     * @param newParity Parity configuration (None, Even, or Odd)
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t setParity(ParityMode newParity);

    /**
     * @brief Set the number of stop bits for UART communication
     *
     * @param newStopBits Stop bits configuration (1, 1.5, or 2 bits)
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t setStopBits(StopBits newStopBits);

    /**
     * @brief Set the flow control mode for data transmission management
     *
     * @param newFlowCtrlMode Flow control configuration (Software, Hardware RTS/CTS)
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t setHW_flowCtrlMode(FlowControl newFlowCtrlMode);

    /**
     * @brief Enable or disable internal pull-up resistor on the RX pin
     *
     * @param enable True to enable pull-up resistor, false to disable (floating)
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_ERR_INVALID_STATE: RX pin not configured (UART_PIN_NO_CHANGE)
     * @note Must be called after init()
     */
    esp_err_t setRxPinInternalPullup(bool enable);

    /**
     * @brief Get the current RX pin internal pull-up resistor state
     *
     * @return bool
     *         - true: Internal pull-up is enabled
     *         - false: Internal pull-up is disabled
     */
    bool getRxPinInternalPullup(void) const;

    /**
     * @brief Set the UART communication mode (RS232, RS485, IrDA)
     *
     * @param newMode UART mode configuration
     * @return esp_err_t
     *         - ESP_FAIL: Not implemented yet
     * @warning This function is not implemented. Always returns ESP_FAIL.
     */
    esp_err_t setUartMode(UartMode newMode);

    // Parameter retrieval functions
    /**
     * @brief Get the current baud rate configuration
     *
     * @param[out] pBaudRate Pointer to store the current baud rate value
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t getBaudRate(int* pBaudRate);

    /**
     * @brief Get the current data word length configuration
     *
     * @param[out] pWordLen Pointer to store the current data bits value
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t getDataBits(WordLength* pWordLen);

    /**
     * @brief Get the current parity mode configuration
     *
     * @param[out] pParity Pointer to store the current parity mode value
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t getParity(ParityMode* pParity);

    /**
     * @brief Get the current stop bits configuration
     *
     * @param[out] pStopBits Pointer to store the current stop bits value
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t getStopBits(StopBits* pStopBits);

    /**
     * @brief Get the current flow control mode configuration
     *
     * @param[out] pFlowCtrlMode Pointer to store the current flow control mode value
     * @return esp_err_t
     *         - ESP_OK: Success
     *         - ESP_FAIL: Parameter error
     */
    esp_err_t getFlowCtrlMode(FlowControl* pFlowCtrlMode);

    /**
     * @brief Get the current UART mode configuration
     *
     * @param[out] pUartMode Pointer to store the current UART mode value
     * @return esp_err_t
     *         - ESP_FAIL: Not implemented yet
     * @warning This function is not implemented. Always returns ESP_FAIL.
     */
    esp_err_t getUartMode(UartMode* pUartMode);

    /**
     * @brief Get the number of bytes available in the RX buffer
     *
     * @return int Number of bytes available to read
     *         - >0: Bytes available
     *         - 0: No data available
     *         - -1: Error (only in event mode)
     * @note Behavior differs based on event handler mode:
     *       - Event mode: Returns dataLen_ from event handler
     *       - Polling mode: Queries hardware buffer length
     */
    int getDataLen(void);

    /**
     * @brief Read data from the UART RX FIFO buffer
     *
     * @param[out] bufferToStore String buffer to store the received data
     * @param msgSize Maximum number of bytes to read (0 = use maxBufferSize_)
     * @return esp_err_t
     *         - ESP_OK: Data successfully read
     *         - ESP_ERR_INVALID_STATE: No data available to read
     *         - ESP_ERR_INVALID_SIZE: Requested size exceeds maxBufferSize_
     *         - ESP_ERR_INVALID_RESPONSE: No data received within timeout
     *         - ESP_FAIL: Read operation failed
     * @note Automatically echoes data if echo mode is enabled
     * @note Clears dataAvailable_ flag after reading
     */
    esp_err_t read(std::string& bufferToStore, size_t msgSize = 0);

    /**
     * @brief Get the last received data from the internal RX buffer
     *
     * @param[out] bufferToStore String buffer to store the last received data
     * @note This retrieves data from the internal rxBuffer_ without reading from
     * hardware. Use read() to fetch new data from the UART peripheral.
     * @note Does not clear the internal buffer or dataAvailable_ flag
     */
    void getData(std::string& bufferToStore);

    /**
     * @brief Write data to the UART TX FIFO buffer for transmission
     *
     * @param[in] bufferToWrite String buffer containing data to transmit
     * @return esp_err_t
     *         - ESP_OK: Data successfully written to TX buffer
     *         - ESP_FAIL: Write operation failed
     */
    esp_err_t write(const std::string& bufferToWrite);

    /**
     * @brief Echo back the last received data through the same UART port
     *
     * @return esp_err_t
     *         - ESP_OK: Echo successful
     *         - ESP_FAIL: Write operation failed
     * @note Also checks if "clear" command was received and clears terminal if so
     * @note Uses data from rxBuffer_ (last read data)
     */
    esp_err_t echo(void);

    /**
     * @brief Enable automatic echo of received data
     *
     * @note When enabled, read() will automatically echo received data
     */
    void enableEcho(void);

    /**
     * @brief Disable automatic echo of received data
     */
    void disableEcho(void);

    /**
     * @brief Check if data is available to read from the RX buffer
     *
     * @return bool
     *         - true: Data is available and ready to read
     *         - false: No new data or data was already retrieved
     * @note Flag is set by event handler (event mode) or getDataLen() (polling mode)
     * @note Flag is cleared by read() after data is retrieved
     */
    bool isDataAvailable(void);

    /**
     * @brief Check if the UART event handler is enabled
     *
     * @return bool
     *         - true: Event handler task is running
     *         - false: Operating in polling mode
     */
    inline bool isEventsEnabled(void) {
        return allowEvents_;
    }

    /**
     * @brief Check if "clear" command was received and clear terminal if found
     *
     * @note Compares first 5 characters of rxBuffer_ with "clear"
     * @note Automatically calls clearTerminal() if match is found
     */
    void isClearCmd(void);

    /**
     * @brief Send a clear screen command to the terminal
     *
     * @note Sends form feed character (0x0C) to clear the terminal screen
     */
    void clearTerminal(void);

    /**
     * @brief Clear the UART RX FIFO hardware buffer
     *
     * @return esp_err_t
     *         - ESP_OK: Buffer successfully cleared
     *         - ESP_FAIL: UART port error
     * @note Only clears hardware buffer, not the internal rxBuffer_ string
     */
    esp_err_t clearRxBuffer(void);

private:
    uart_port_t port_;
    int txPin_;
    int rxPin_;
    int rtsPin_;
    int ctsPin_;
    int baudRate_;
    bool rxInternalPullup_;  ///< Internal pull-up resistor enable for RX pin

    uint16_t maxBufferSize_;  ///< Rx buffer size to allocate

    int16_t dataLen_;  ///< Received message size

    std::string rxBuffer_;  ///< Buffer to store the received data

    bool isHW_flowControlSet = false;  ///< Hardware flow control flag

    uint8_t rxThresh_;  ///< Defines the word limit to set the rx FIFO higher
                        ///< watermark interrupt by hardware or software
    static const uint16_t kDefaultQueueSize = 10;  ///< Default queue size
    uint16_t queueSize_;  ///< When event handler is enabled, it handles how
                          ///< much messages will be waiting in the queue

    QueueHandle_t QueueHandle_;  ///< Queue to handle received messages

    uint16_t stackSize_ = 4096;  ///< Event handler task stack size

    bool allowEvents_;  ///< When it is true, event handler is created and
                        ///< enabled

    bool dataAvailable_ = false;  ///< To know if there is data stored
                                  ///< in rxBuffer_

    bool echoEnable_ = false;  ///< When it is true, echo function is enabled

    const char clearingChar_ = '\x0C';  ///< This character clears the screen
                                        ///< when using a terminal by uart. It
                                        ///< is use by clearTerminal function.

    std::string taskName_;               ///< Store the unique event handler task name
    TaskHandle_t taskHandle_ = nullptr;  ///< Task handle for the event handler
    /**
     * @brief Static task function to handle UART events triggered by interrupts
     *
     * @param[in] pvParameters Pointer to UART object (passed as void*)
     * @note Runs as FreeRTOS task with priority 12 and 4KB stack
     * @note Handles events: UART_DATA, UART_BUFFER_FULL, UART_FIFO_OVF, etc.
     * @note Polls event queue every 100ms
     */
    static void eventHandler(void* pvParameters);
};
