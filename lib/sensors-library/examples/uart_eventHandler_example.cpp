/*******************************************************************************
 * @file uart_test.hpp
 * @brief Contains the test example for UART using espidf, event handler is the
 * default mode!!!
 *
 * @version 0.1.0
 * @date 2025-03-27
 * @author mateor@sense-ai.co - Sense AI
 *******************************************************************************
 *******************************************************************************/
#include "uart_sense.hpp"

constexpr uart_port_t kUartPort = UART_NUM_0;

// The pins are received as int type values. Change them as needed.
constexpr int kPinTX = UART_PIN_NO_CHANGE;
constexpr int kPinRX = UART_PIN_NO_CHANGE;
constexpr int kPinRTS = UART_PIN_NO_CHANGE;
constexpr int kPinCTS = UART_PIN_NO_CHANGE;

// Those are the default values for UART parameter. Change them as needed.
constexpr int kBaudRate = 115200;

// This will be the size of the rx buffer to receive data
constexpr uint16_t kBufferSize = 1024;

// This will be the max number of received messages to handle by the event
// handler, if more messages than allowed are received it will lead to a
// queue error. The max size of the queue is the max number of slots in the rx
// FIFO buffer (128)
constexpr uint16_t kQueueSize = 10;

// Full constructor
// UART uart0(kUartPort, kPinTX, kPinRX, kPinRTS, kPinCTS,
//     kBaudRate, kBufferSize, kQueueSize, true);

// Default constructor
UART uart0(kBaudRate);

// Port constructor
// UART uart0(kUartPort,kBaudRate);

extern "C" void app_main() {
    esp_err_t err = uart0.init();

    if (err) {
        printf("Error in UART config: %s\n", esp_err_to_name(err));
    } else {
        printf("UART initialized\n");
    }
    // Allow echo if needed
    uart0.enableEcho();

    uart0.write("Welcome to the example of the UART class\n");

    while (1) {
        if (uart0.isDataAvailable()) {  // Use it to know when new data is
                                        // available
            std::string str1;
            uart0.read(str1);  // Read all available data

            static const size_t kStr1_initialIndex = 0;
            static const size_t kStr1_strLen = 6;
            static const size_t kStr2_initialIndex = 0;
            static const size_t kStr2_strLen = 6;

            std::string str2("reboot");

            if (str1.compare(kStr1_initialIndex, kStr1_strLen, str2,
                             kStr2_initialIndex, kStr2_strLen) == 0) {
                uart0.write("Restarting in 5s...\n");
                vTaskDelay(pdMS_TO_TICKS(5000));
                esp_restart();  // Just as an example...
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}