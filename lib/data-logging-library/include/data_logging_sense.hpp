/*******************************************************************************
 * @file data_logging_sense.hpp
 * @brief Contains the classes and method definitions to work logging/debugging
 *
 * @version v0.2.0
 * @date 2024-05-03
 * @author Sense-AI
 *******************************************************************************/

#pragma once

#include <cstdint>

/**
 * @class Debugger
 * @brief A utility class for conditional debugging output.
 *
 * This class provides methods for printing debugging information conditioned
 * on an internal flag, allowing for easy enable/disable of debugging output.
 */
class Debugger {
public:
    /**
     * @brief Constructs a Debugger object with optional debugging active flag.
     * @param activeFlag Initializes the debug activity state. True enables
     *                   debugging output, false disables it.
     */
    Debugger(bool activeFlag);

    /**
     * @brief Destructor for the Debugger class.
     */
    ~Debugger();

    /**
     * @brief Prints a string if debugging is active.
     * @param string The C-style string to print.
     */
    void print(const char* string);

    /**
     * @brief Prints a single character if debugging is active.
     * @param character The character to print.
     */
    void print(const char character);

    /**
     * @brief Prints an unsigned 8-bit integer if debugging is active.
     * @param value The uint8_t value to print.
     */
    void print(const uint8_t value);

    /**
     * @brief Prints an unsigned 16-bit integer if debugging is active.
     * @param value The uint16_t value to print.
     */
    void print(const uint16_t value);

    /**
     * @brief Prints an unsigned 32-bit integer if debugging is active.
     * @param value The uint32_t value to print.
     */
    void print(const uint32_t value);

    /**
     * @brief Prints an unsigned 64-bit integer if debugging is active.
     * @param value The uint64_t value to print.
     */
    void print(const uint64_t value);

    /**
     * @brief Prints a signed 8-bit integer if debugging is active.
     * @param value The int8_t value to print.
     */
    void print(const int8_t value);

    /**
     * @brief Prints a signed 16-bit integer if debugging is active.
     * @param value The int16_t value to print.
     */
    void print(const int16_t value);

    /**
     * @brief Prints a signed 32-bit integer if debugging is active.
     * @param value The int32_t value to print.
     */
    void print(const int32_t value);

    /**
     * @brief Prints a signed 64-bit integer if debugging is active.
     * @param value The int64_t value to print.
     */
    void print(const int64_t value);

    /**
     * @brief Prints formatted text if debugging is active.
     *
     * This method mimics the printf function and allows for formatted output.
     * @param format The format string (printf-style) that specifies how
     *               subsequent arguments are converted for output.
     * @param ... Variable arguments to be formatted according to the format
     *            string.
     */
    void printf(const char* format, ...);

    /**
     * @brief Prints a string followed by a newline and carriage return
     *        if debugging is active.
     * @param string The C-style string to print.
     */
    void println(const char* string);

    /**
     * @brief Prints a single character followed by a newline and carriage
     *        return if debugging is active.
     * @param character The character to print.
     */
    void println(const char character);

    /**
     * @brief Prints an unsigned 8-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The uint8_t value to print.
     */
    void println(const uint8_t value);

    /**
     * @brief Prints an unsigned 16-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The uint16_t value to print.
     */
    void println(const uint16_t value);

    /**
     * @brief Prints an unsigned 32-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The uint32_t value to print.
     */
    void println(const uint32_t value);

    /**
     * @brief Prints an unsigned 64-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The uint64_t value to print.
     */
    void println(const uint64_t value);

    /**
     * @brief Prints a signed 8-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The int8_t value to print.
     */
    void println(const int8_t value);

    /**
     * @brief Prints a signed 16-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The int16_t value to print.
     */
    void println(const int16_t value);

    /**
     * @brief Prints a signed 32-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The int32_t value to print.
     */
    void println(const int32_t value);

    /**
     * @brief Prints a signed 64-bit integer followed by a newline and
     *        carriage return if debugging is active.
     * @param value The int64_t value to print.
     */
    void println(const int64_t value);

private:
    bool debugActive_;  ///< Flag indicating whether debugging output is enabled.
};
