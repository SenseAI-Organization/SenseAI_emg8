<div align="center">

# C/C++ Code Structure and Style (C++17)

[![Version](https://img.shields.io/badge/version-1.2.0-blue?style=flat-square)](../changelog/CHANGELOG.md)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat-square&logo=cplusplus&logoColor=white)]()

**Sense AI standards for C and C++ codebases**

</div>

---

> For Python guidelines, see [Python Code Structure and Style](./Python_CodeStyle.md)

---

## Table of Contents

1. [Header Files](#1-header-files)
2. [Scoping](#2-scoping)
3. [Classes](#3-classes)
4. [Functions/Methods](#4-functionsmethods)
5. [C++ Features](#5-c-features)
6. [Naming](#6-naming)
7. [Comments and Documentation](#7-comments-and-documentation)
8. [Formatting](#8-formatting)
9. [Embedded Systems Rules](#9-embedded-systems-rules)
10. [Code Quality Tooling](#10-code-quality-tooling)
11. [Unit Testing Conventions](#11-unit-testing-conventions)

---

## 1. Header Files

In general, every .cc (.c, .cpp) file should have an associated .h (.hpp) header file. The only exceptions are files intended for unit tests or containing a short main_app() function.

* **Avoid Multiple Inclusion:** Use **#pragma once** to prevent multiple inclusion of the same header file. This is preferred over include guards due to compatibility with the Espressif framework.  
* **Self-Contained Headers**: Each header file must be self-contained—it should compile correctly on its own. This means it must include all header files it directly depends on and should not require specific files to be included before it.  
* **Include Order**: To reduce hidden dependencies and ensure headers are self-contained, follow this specific order when including header files:  
  1. The corresponding header file (.h/.hpp) for the .cpp file (e.g., if editing foo.cpp, foo.h is included first).  
  2. **C** standard library headers in the form <file.h>.  
  3. **C++** standard library headers in the form <file>.  
  4. ESP-IDF libraries (such as esp_log, esp_system, esp_sleep, etc.).  
  5. FreeRTOS headers.  
  6. Third-party library headers not part of the project, using double quotes.  
  7. Project header files, using double quotes "file.h".

Separate each group of includes with a blank line to improve readability and make the separation explicit. Example for fooserver.c:

```c
#include <sys/types.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "esp_log.h"
#include "esp_timer.h"

#include "third_party/absl/flags/flag.h"

#include "base/basictypes.h"
#include "foo/server/bar.h"
```

* **Avoid Relative Paths in Includes**: When including header files from the current project, prefer paths based on the project directory structure rather than relative paths. Avoid using "./" and "../". This makes includes clearer and less error-prone if the directory structure changes.

## 2. Scoping

* **Minimize Use of Globals**: Avoid global variables due to problems they cause (difficulty tracking program state, name collisions). If necessary, restrict scope as much as possible and consider alternatives such as design patterns that encapsulate data more safely.  
* **Use of Namespaces**: Use **namespaces** to group related code and avoid name conflicts.  
- Avoid **using-directives** at global scope in header files (e.g., "using namespace foo").  
- **using-declarations** (e.g., "using foo::FooBar") may be used carefully. Place them inside functions or methods to limit scope.  
- Define functions within **namespaces** unless intended as extensions of existing classes/namespaces.  
- For file-level objects or functions, prefer **anonymous namespaces** over **static**. Code inside namespaces should **not be indented**.  
* **Variable Lifetime**: Emphasize limiting variable lifetime to the minimum necessary. Ideally, declare a variable in the innermost block allowing its complete use ({ }).  
* **Variable Initialization**: Initialize variables as soon as they are declared. Declare them as close to first use as possible. Use initialization methods that make code most readable and explicit about intent.  
* **Function and Method Scope**: Keep functions and methods short and focused. If a function performs multiple distinct or complex operations, decompose it into smaller functions with clearly defined purposes (see [Functions](#4-functionsmethods) section).

## 3. Classes

* **Accessibility**: Class members should be private by default. Exceptions: Constants requiring user access (public), interface members (protected).  
* **Constructors**: Should be limited to member initialization. If complex initialization is needed, use a factory or initialization method. Constructors should avoid calling virtual methods.  
* **Inheritance**: Use inheritance with caution; prefer composition over inheritance.  
  - Inheritance (which must be public) should only be used to represent an is-a relationship (e.g., an accelerometer is a sensor).   
  - Multiple inheritance should be avoided.  
  - For interface inheritance, use pure abstract classes (classes with at least one pure virtual member function).  
  - When using polymorphism, ensure destructors are virtual.  
* **Member Declaration Order**: Class members should be declared in the following order:  
  1. Public.  
  2. Protected.  
  3. Private.

   Within each section, group elements as follows:

  1. **Types** and **type aliases** (**typedef**, **using**, **enum**, nested structures and classes, and **friend types**).  
  2. (Optionally, for structs only) **non-static** data members.   
  3. **static** constants.  
  4. **Factory** functions.  
  5. Constructors and assignment operators.   
  6. Destructor.  
  7. All other members (**static** and **non-static**).  
  8. All other functions (**static**, **non-static**, **friend**).  
* **Operator Overloading**: Operator overloading should be intuitive and avoid surprises, following established conventions (such as + for addition). Overloading operators for other purposes is discouraged.

## 4. Functions/Methods

* **Input and Output**   
    - **Hardware-interfacing functions** (e.g., peripherals, antennas, connections) must return **esp_err_t** with outputs passed by reference using clear, descriptive parameter names (e.g., `temperature`, `result`, `data`). Mark output parameters in Doxygen with `@param[out]`.  
    - **Non-hardware functions** should prefer **esp_err_t** for consistency. However, **bool** or custom enums are acceptable if they make code clearer. When using enums, always include a "success" state and an "error/unknown" state.  
    - Prefer returning by value when possible, or by reference as an alternative. Avoid returning pointers unless returning **nullptr** is a valid case.  
    - When functions have both input and output parameters, order input-only parameters (`@param[in]`) before outputs (`@param[out]`).  

  **Example:**
  ```cpp
  /**
   * @brief Reads temperature from sensor.
   * @param[in] sensorId - sensor identifier (0-7)
   * @param[out] temperature - pointer to store temperature (°C)
   * @return ESP_OK on success, ESP_ERR_INVALID_ARG if temperature is nullptr
   */
  esp_err_t readTemperature(uint8_t sensorId, float* temperature);
  ```  
* **Write Short Functions**: Follow the philosophy: One action leads to one function. Functions should not exceed 50 lines of code.  
- Long functions are permitted in some cases. However, always attempt to decompose them into smaller functions without breaking functionality.  
* **Function Overloading**: Function/method overloading (including constructors) is permitted and encouraged for both type and argument count variations. However, overloading must not change the function's semantics between different "options".  
* **Default Arguments**: Allowed only in non-virtual functions, and only when the default value is guaranteed to remain constant (not dependent on external factors). 

## 5. C++ Features

* **C++17 Features**: We use **C++17 standard**. Encouraged features include:  
  - Structured bindings: `auto [status, value] = fetchData();`  
  - if constexpr for compile-time branching  
  - std::optional for optional values  
  - std::variant for type-safe unions  
  - **Avoid C++20+ features** (std::bit_cast, modules, etc.)  
* **Exceptions**: Exceptions are **banned** for firmware development. They introduce unpredictable control flow and code size overhead unsuitable for constrained embedded systems. Use error codes (esp_err_t, bool, or custom enums) instead.  
* **Strings**:  
  - Work with strings **C-style** using `char*` and `const char*` for compatibility and efficiency.  
  - **Allowed**: `std::string_view`, `std::span`, `std::array`, `absl::Span` (read-only or fixed-size containers).  
  - **Discouraged**: `std::string` (use only if justified and documented; prefer stack buffers or string_view).  
  - **Banned**: C++ iostreams (`<iostream>`, `std::cout`, `std::stringstream`) due to code size and runtime overhead.  
* **Casting**: Do not use C-style casts. Use C++ cast operators when explicit type conversion is required:  
  - Use brace initialization for arithmetic type conversions (e.g., **int64_t{x}**).  
  - Use **static_cast** for value-converting casts, explicit upcasts from derived to base class, or explicit downcasts from base to derived class.  
  - Use **const_cast** to remove the **const** qualifier (sparingly and with justification).  
  - Use **reinterpret_cast** for unsafe pointer type conversions. Only use if you understand aliasing and document it.  
  
  **Example:**
  ```cpp
  // Correct - Use static_cast
  int value = 42;
  float dValue = static_cast<float>(value);
  
  // Correct - Brace initialization
  uint32_t mask = uint32_t{0xFF};
  
  // Wrong - C-style cast
  float dValue = (float)value;
  ```  
* **Increment and Decrement**: Use prefix form (e.g., ++i) instead of postfix (e.g., i++). Postfix form is only permitted when its specific semantics are needed.  
* **Use of const**: Use const (or constexpr when the compiler requires it) when a variable will be constant and it makes sense, especially if:  
  - A function passes a value **by reference** and will not change it or its members. If passing by value, avoid using const.  
  - A method does not modify object state, or does not allow the user to modify it.  
* **Integers**: Avoid "long", "short" and instead use typedef indicating bit count (e.g., **int8_t**, **int16_t**). Declare all variables as signed integers (even if not expected to take negative values) unless they represent organized bits (such as register masks), in which case they should be declared as unsigned (e.g., uint8_t, uint16_t). If the maximum value is known, define in the smallest possible size (bytes); when in doubt, use a larger size.  
* **Null Pointer**: Use **nullptr** for pointers, **'\0'** for the null character (note the backslash).  
* **Structure Initialization**: If initializing structures with specific element values, do so in the order elements were defined to avoid compatibility issues.  
* **Switch-case**: Must always have a default case (**default**). If the default case should not occur, handle **default** as an error. 

## 6. Naming

These are among the most important consistency rules. The style of a name should immediately convey the type of entity being named (type, variable, function, class, etc).

**Consistency is more important than individual preferences.** Regardless of whether a programmer likes them or not, the rules are the rules. **All** code must be written in **English**, including documentation.

### Naming Convention Summary

| Entity | Convention | Example |
|--------|-----------|---------|
| **Files** | snake_case (lowercase) | `sensor_control.cpp`, `temperature_sensor.hpp` |
| **Types** (classes, structs, enums) | PascalCase | `TemperatureSensor`, `SensorConfig` |
| **Typedefs** | PascalCase with _t suffix | `sensor_config_t` |
| **Variables** | camelCase | `sensorId`, `temperatureValue` |
| **Constants** | kCamelCase (k prefix) | `kMaxSensors`, `kDefaultTimeout` |
| **Class Members** | camelCase with _ suffix | `temperature_`, `sensorId_` |
| **Static Variables** | s_ prefix + camelCase | `s_instanceCount`, `s_initFlag` |
| **Functions/Methods** | camelCase | `readTemperature()`, `initSensor()` |
| **Namespaces** | snake_case | `sensor_control`, `data_processing` |
| **Enumerators** | kCamelCase (k prefix) | `kOk`, `kOutOfMemory`, `kError` |
| **Macros** | UPPER_CASE | `MAX_BUFFER_SIZE`, `LOG_TAG` |

### Detailed Rules
- Abbreviations are prohibited (unless standard and listed in Wikipedia). In some cases of limited scope (such as within a 5-line function), it's fine to use letters for accumulators, counters.  
- Names must be clear even to people from other teams.  
- Use names that describe the object's purpose; don't worry about saving horizontal space. It's more important that others understand the code.  
* **Files**: Use snake_case, all lowercase. Additionally, use .cpp for C++ files and .hpp for headers.  
* **Types (classes, structures, enums)**: Use PascalCase. If the structure is defined as typedef, indicate with a _t suffix.  
* **Variables**: Use camelCase. Use '_' only when there may be conflicts with capitalization and the separator improves readability.  
- Constants should be indicated as kVariableName (with k at the beginning).  
- Variables that are class members must end with '_' (e.g., `temperature_`, `buffer_`).  
- Variables within structures are written like normal variables (i.e., camelCase).  
- **static** variables must begin with 's_' prefix for easy identification (e.g., `s_instanceCount`, `s_initFlag`).  
- **Function output parameters** should use clear, descriptive names and be marked with `@param[out]` in Doxygen (e.g., `temperature`, `result`, `data`).  

  **Example:**
  ```cpp
  class TemperatureSensor {
  private:
      float temperature_;              // Class member with _ suffix
      static int s_instanceCount_;     // Static member with s_ prefix
  
  public:
      /** 
       * @brief Reads current temperature.
       * @param[out] temperature - temperature in °C
       * @return ESP_OK on success, ESP_ERR_NOT_READY if sensor not ready
       */
      esp_err_t getTemperature(float* temperature);
  };
  ```
* **Functions and Methods**: Like variables, these must be written in camelCase.  
* **Namespaces**: These should be descriptive, using snake_case. Highest-level namespaces should be based on the project name. Avoid names that may cause internal collisions.

Code written inside a namespace should not be indented.

* **Enumerators**: Should be written as if they were constants, not macros—i.e., using kCamelCase. Additionally, evaluate when it's appropriate to define the list as **class** or in a simple manner. Example:

```c
enum class UrlTableError {
  kOk = 0,
  kOutOfMemory,
  kMalformedInput,
};
```

* **Macros**: In general, macros are **NOT** recommended, but if necessary, write in all uppercase, using '_' to separate. Ideally, limit to using macros provided by the Espressif framework.

## 7. Comments and Documentation

In general, avoid obvious comments. Avoid commenting on what the code does, and instead indicate why it does what it does. **All** comments must be written in **English**. Additionally, **documentation must follow the Doxygen format**, presented in [Doxygen: Documenting the code](https://www.doxygen.nl/manual/docblocks.html).

* **Files**: Each .h (or .hpp) file must begin with a comment block indicating the license used in the project. If this block is properly written, associated files (.cpp and test) should not carry the comment block. If there is a file without an associated .h, it must have the license comment.  
* **Line Comments**: Use // for line comments.  
* **Documentation**: Documentation within code is mandatory. Additionally, use /** */, indicating relevant information with @. Example:

```c
/**
* @brief A description of the element. 
*/
// element
```

* **Class Documentation**: Classes (as well as structures and lists) must have corresponding documentation, providing sufficient information about how and when the defined entity should be used, plus additional considerations for its proper use.  
* **Function Comments:** There are two types: declaration comments (which are part of documentation) and definition comments. Declaration comments usually go in the .h file. If a function is declared in the .cpp, it carries its respective declaration comment. These rules also apply to methods (private and public).  
* **Function Declaration**: This comment should include:  
    - Description (brief). Additionally, implicitly begin with "This function…". (E.g., *Resolves the errors with the calibration process*.)  
    - Inputs (param). Use @param[in] for input parameters, @param[out] for output parameters.  
    - Outputs (return). Use @return to describe return values.  
    - Additional note about dependencies or characteristics for using the function properly, if necessary. Use @note.  
* **Function Definition**: These should only be included when there is something the reader must know about what the function does (i.e., the code written to implement the function).  
* **Variable Comments**: In general, variable names should be sufficiently descriptive. Only include additional comments when clarification of non-obvious aspects is needed. Additionally, code blocks that may be difficult to understand at a glance should carry an explanatory comment. Global variables must carry a comment explaining what they are used for and/or why they must be global.

## 8. Formatting

* **Line Length:** 90 characters is the maximum allowed per line. A line may exceed 90 characters if it is:  
  - A comment line that is not feasible to split without impairing readability, cut-and-paste ease, or auto-linking — for example, if a line contains an example command or a URL literal longer than 90 characters.  
  - A string literal that cannot be easily split into 90 columns. This may be because it contains URIs or other semantically critical pieces, or because the string contains embedded language, or a multi-line string whose line breaks are significant such as help messages. In such cases, splitting the string would reduce readability, searchability, clickability, etc. Except for test code, such strings should appear in namespace scope near the top of a file.  
  - An include statement.  
  - A using statement.  
* **Spacing:**   
  - The words "public, protected, private" within the class must be at the same indentation level as the class.  
  - Inside a namespace there should be no indentation (i.e., code inside a namespace should be at the same level as the namespace).  
  - Functions should **not** have a space line after the function name.  
  - Functions must have a space before the opening brace '{'.  
  - Loops and conditionals must have a space before the parenthesis '(' and before the opening brace '{'. Additionally, in conditional blocks with more than one condition (**if**), the next condition (**else if** or **else**) must start on the same line where the previous condition ended, with a space between the closing brace '}' and the condition (**else if** or **else**).

The following code block exemplifies some of the mentioned rules:

```cpp
namespace sensor_control {
// No indentation inside namespace

class TemperatureSensor {
public:  // No indentation, same level as class
    TemperatureSensor(uint8_t sensorId);
    
    /**
     * @brief Reads current temperature.
     * @param[out] temperature - temperature in °C
     * @return ESP_OK on success, ESP_ERR_NOT_READY if not initialized
     */
    esp_err_t readTemperature(float* temperature);

protected:  // No indentation
    esp_err_t validateSensor();

private:  // No indentation
    uint8_t sensorId_;
    float calibrationOffset_;
    static int s_totalSensors_;
};

esp_err_t initializeSensors() {
    if (xSemaphoreTake(sensorMutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Code inside conditional
    } else if (/* retry logic */) {
        // Next condition on same line as }
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

}  // namespace sensor_control
```

## 9. Embedded Systems Rules

At Sense AI, we follow the best programming practices proposed by NASA for embedded systems, summarized in 10 key rules:

1. **Standardize Programming Approaches**: Adopt uniform programming methodology to improve code coherence and quality. This includes the use of coding standards, common tools, and techniques that facilitate code understanding and maintenance.  
2. **Limit Source Code Use**: Restrict code to what is absolutely necessary, avoiding unnecessary complexity. This helps reduce errors and facilitates software verification and validation.  
3. **Restrictions on Software Use**: Apply strict policies on the inclusion of third-party software to ensure all integrated software meets required security and reliability standards.  
4. **Peer Code Review**: Implement a peer code review process to detect and correct errors. This collaborative approach increases code quality and development efficiency.  
5. **Verification of Input/Output Data Accuracy**: Ensure all input and output data are valid and within expected ranges to prevent errors and unexpected system behavior.  
6. **Limitation of Dynamic Memory Use**: Avoid or minimize dynamic memory allocations after initialization to prevent memory fragmentation and out-of-memory errors.  
   - Prefer static allocation or allocation at system startup.  
   - Use `std::array`, fixed-size buffers, or pre-allocated memory pools instead of `new` or `malloc`.  
   - Document any post-initialization dynamic allocation with justification.  
   - Be aware that frequent large allocations can cause heap fragmentation on embedded systems.  
7. **Understanding and Control of Execution Flow**: Maintain strict control over software execution flow, avoiding practices such as excessive event-driven programming that can make control flow difficult to follow and predict.  
8. **Race Conditions and Concurrency**: Prevent race conditions and concurrency issues by ensuring software is safe in parallel or multitasking execution environments. Use appropriate synchronization primitives (mutexes, atomic operations).  
9. **Variable Initialization**: Initialize all variables before use to prevent errors related to undefined or residual values.  
10. **Software Limitations and Exceptions**: Properly establish and handle software operational limits, including exception/error handling to ensure the system can recover in a controlled manner from anomalous situations.  

### Additional Embedded-Specific Guidelines

* **Interrupt Service Routines (ISRs)**: Keep ISRs short and simple. Avoid complex operations, heap allocations, and potentially blocking calls. Use volatile for shared data modified in ISRs.  
* **Watchdog and Timeouts**: Implement watchdog timers and reasonable timeouts for critical operations to detect and recover from system hangs or deadlocks.  
* **Logging Levels in ISRs**: Minimize or avoid logging in ISRs; if necessary, use high-speed non-blocking logging.  
* **Volatile and Atomic Types**: Use `volatile` for hardware registers and variables modified in ISRs or accessed from multiple threads. Use `std::atomic` for safe concurrent access to shared data.  

## 10. Code Quality Tooling

**Build System Note**: All CMake configuration is handled by **PlatformIO** (via `platformio.ini`). Developers should never modify CMakeLists.txt directly. For Git workflows, branching strategy, and commit message conventions, see [Git Workflow Guide](../workflow/GitWorkflow.md).

Automated tools help enforce these standards consistently across the codebase:

* **clang-format**: Enforces consistent code formatting (indentation, spacing, line breaks). Configuration file `.clang-format` should be present in the repository root. Run before commits:

  Format C++ and header files:
  ```bash
  clang-format -i *.cpp *.hpp
  ```

* **clang-tidy**: Performs static analysis to identify bugs, performance issues, and style violations. Configuration file `.clang-tidy` should be in the repository root. Integrate into the build pipeline:

  Run static analysis on C++ files:
  ```bash
  clang-tidy -p=build *.cpp -- -I./include
  ```

* **cppcheck**: Detects errors in C/C++ code, including memory leaks, buffer overflows, and logic errors. Run periodically:

  Run static analysis with all checks enabled:
  ```bash
  cppcheck --project=compile_commands.json --enable=all --suppress=missingIncludeSystem .
  ```
  Create a `.cppcheck` config file in the repository to define project-specific rules and suppressions.

All pull requests must pass these tool checks before being merged.

## 11. Unit Testing Conventions

Unit testing is a critical best practice for reliable embedded firmware. Comprehensive testing patterns, frameworks (Google Test, Catch2), mocking strategies, and CI integration are documented in the [Unit Testing Guide](../testing/UnitTestingGuide.md).

### Quick Testing Reference

* **Test File Naming**: `test_<module>.cpp` for unit tests  
* **Test Organization**: One test fixture per source module  
* **Test Naming**: `test_<function>_<condition>_<expected_result>` (e.g., `test_readTemperature_invalidSensor_returnsError`)  
* **Test Structure (Arrange-Act-Assert)**:  
  ```cpp
  TEST(TemperatureSensorTest, ReadTemperatureReturnsValidValue) {
      // Arrange: Set up test data and mocks
      TemperatureSensor sensor(1);
      
      // Act: Execute the function
      float temperature = 0.0f;
      esp_err_t result = sensor.readTemperature(&temperature);
      
      // Assert: Verify results
      ASSERT_EQ(result, ESP_OK);
      ASSERT_GE(temperature, -40.0f);
      ASSERT_LE(temperature, 125.0f);
  }
  ```
* **Coverage Goals**: Aim for >80% code coverage; prioritize critical error paths and ISR code  
* **No Hardware**: Mock all ESP-IDF calls; use dependency injection for testability  
* **Run Tests**: Via PlatformIO testing environment; all tests must pass before merging  

---

## 📚 Resources

| Resource | Description |
|----------|-------------|
| [Google Test Documentation](https://google.github.io/googletest/) | Official C++ testing framework docs |
| [Unit Testing Guide](../testing/UnitTestingGuide.md) | Sense AI C++ unit testing practices |
| [Cppcheck Guide](../tooling/CppcheckGuide.md) | Static analysis configuration |
| [Doxygen Manual](https://www.doxygen.nl/manual/) | Code documentation standard (C++) |
| [clang-format](https://clang.llvm.org/docs/ClangFormat.html) | Automatic code formatting |
| [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) | Static analysis linter |
| [Changelog](../changelog/CHANGELOG.md) | Documentation version history |

---

<div align="center">

**Part of Sense AI Documentation**

[⬅️ Back to Documentation](../README.md)

</div>
