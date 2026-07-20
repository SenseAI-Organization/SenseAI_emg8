/*******************************************************************************
 * @file uart_test.hpp
 * @brief Contains the test example for UART using espidf
 *
 * @version 0.1.0
 * @date 2025-03-27
 * @author mateor@sense-ai.co - Sense AI
 *******************************************************************************
 *******************************************************************************/
#include "uart_sense.hpp"

constexpr uart_port_t kUartPort = UART_NUM_0;
constexpr int kGpioNum43 = 43;
constexpr int kGpioNum44 = 44;
constexpr int kPinTX = kGpioNum43;
constexpr int kPinRX = kGpioNum44;
constexpr int kPinRTS = UART_PIN_NO_CHANGE;
constexpr int kPinCTS = UART_PIN_NO_CHANGE;

constexpr int kBaudRate = 115200;

// This will be the size of the rx buffer to receive data
constexpr uint16_t kBufferSize = 1024;

// This will be the max number of received messages to handle by the event
// handler, if more messages than allowed are received it will lead to a
// queue error. The max size of the queue is the max number of slots in the rx
// FIFO buffer (128)
constexpr uint16_t kQueueSize = 10;

UART uart0(kUartPort, kPinTX, kPinRX, kPinRTS, kPinCTS, kBaudRate, kBufferSize,
           kQueueSize, false);

extern "C" void app_main() {
    esp_err_t err = uart0.init();
    if (err) {
        printf("Error in UART config: %s", esp_err_to_name(err));
    }

    // Enable internal pull-up resistor on RX pin (call after init)
    err = uart0.setRxPinInternalPullup(true);
    if (err) {
        printf("Error setting RX internal pull-up: %s\n", esp_err_to_name(err));
    }

    uart0.write("Welcome to the UART class example!\n");

    while (1) {
        int len = uart0.getDataLen();
        if (len < 1) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        std::string dataRetrieved;
        err = uart0.read(dataRetrieved);
        if (err) {
            printf("Error in UART reading: %s\n", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // This is an echo as the function echo() but since the buffer is
        // already read it is not reliable
        uart0.write(dataRetrieved);

        // As an example, this is a command called when "clear" is wrote
        uart0.isClearCmd();

        // This is an example of string comparison
        static const size_t kComparedStr_initialIndex = 0;
        static const size_t kComparedStr_strLen = 6;
        static const size_t kCompareStr_initialIndex = 0;
        static const size_t kCompareStr_strLen = 6;

        std::string comparedStr = dataRetrieved;
        std::string compareStr("reboot");
        if (comparedStr.compare(kComparedStr_initialIndex, kComparedStr_strLen,
                                compareStr, kCompareStr_initialIndex,
                                kCompareStr_strLen) == 0) {
            uart0.write("Restarting in 5s...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();  // Just as an example...
        }
    }
}
