/*******************************************************************************
 * @file data_logging_sense.cpp
 * @brief Contains the classes and method declarations to work logging/debugging
 *
 * @version v0.2.0
 * @date 2024-05-03
 * @author Sense-AI
 *******************************************************************************/
#include <cstdarg>
#include <cstdio>  // For vsnprintf

#include "data_logging_sense.hpp"

Debugger::Debugger(bool activeFlag = true) : debugActive_(activeFlag) {
    // Placeholder
}

Debugger::~Debugger() {
}

void Debugger::print(const char* string) {
    if (debugActive_) {
        printf("%s", string);
    }
}

void Debugger::print(const char character) {
    if (debugActive_) {
        printf("%c", character);
    }
}

void Debugger::print(const uint8_t value) {
    if (debugActive_) {
        printf("%u", value);
    }
}

void Debugger::print(const uint16_t value) {
    if (debugActive_) {
        printf("%u", value);
    }
}

void Debugger::print(const uint32_t value) {
    if (debugActive_) {
        printf("%lu", value);
    }
}

void Debugger::print(const uint64_t value) {
    if (debugActive_) {
        printf("%llu", value);
    }
}

void Debugger::print(const int8_t value) {
    if (debugActive_) {
        printf("%d", value);
    }
}

void Debugger::print(const int16_t value) {
    if (debugActive_) {
        printf("%d", value);
    }
}

void Debugger::print(const int32_t value) {
    if (debugActive_) {
        printf("%ld", value);
    }
}

void Debugger::print(const int64_t value) {
    if (debugActive_) {
        printf("%lld", value);
    }
}

void Debugger::printf(const char* format, ...) {
    if (debugActive_) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
}

void Debugger::println(const char* string) {
    if (debugActive_) {
        printf("%s\n\r", string);
    }
}

void Debugger::println(const char character) {
    if (debugActive_) {
        printf("%c\n\r", character);
    }
}

void Debugger::println(const uint8_t value) {
    if (debugActive_) {
        printf("%u\n\r", value);
    }
}

void Debugger::println(const uint16_t value) {
    if (debugActive_) {
        printf("%u\n\r", value);
    }
}

void Debugger::println(const uint32_t value) {
    if (debugActive_) {
        printf("%lu\n\r", value);
    }
}

void Debugger::println(const uint64_t value) {
    if (debugActive_) {
        printf("%llu\n\r", value);
    }
}

void Debugger::println(const int8_t value) {
    if (debugActive_) {
        printf("%d\n\r", value);
    }
}

void Debugger::println(const int16_t value) {
    if (debugActive_) {
        printf("%d\n\r", value);
    }
}

void Debugger::println(const int32_t value) {
    if (debugActive_) {
        printf("%ld\n\r", value);
    }
}

void Debugger::println(const int64_t value) {
    if (debugActive_) {
        printf("%lld\n\r", value);
    }
}
