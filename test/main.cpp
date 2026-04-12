/*******************************************************************************
 * main.cpp
 * 
 * Main program for testing sensor libraries.
 * Sense-AI
********************************************************************************
*******************************************************************************/
#include <mutex>
#include <cstring>
#include <time.h>
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "ADS1015.hpp"
#include "actuators_sense.hpp"
#include "switch_sense.hpp"
#include "sd_storage_sense.hpp"
#include "spi_gp_sense.hpp"
#include "rgb_led.hpp"

// I2C configs
constexpr gpio_num_t kSDA1 = GPIO_NUM_45;
constexpr gpio_num_t kSCL1 = GPIO_NUM_47;
I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 400000, false);

constexpr gpio_num_t kSDA0 = GPIO_NUM_6;
constexpr gpio_num_t kSCL0 = GPIO_NUM_7;
I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 400000, false);
//-------------------------------------------------

// SD Card configs
constexpr gpio_num_t kCsPin = GPIO_NUM_10;
constexpr gpio_num_t kSclPin = GPIO_NUM_12;
constexpr gpio_num_t kMosiPin = GPIO_NUM_11;
constexpr gpio_num_t kMisoPin = GPIO_NUM_13;

SPI* spiMaster = nullptr;
SD* sdCard = nullptr;
//------------------------------------------------- 

// Button and LED pins
constexpr gpio_num_t kButtonPin = GPIO_NUM_9;
constexpr gpio_num_t kLedPin = GPIO_NUM_2; // RGB LED pin

// System state variables
volatile bool isRecording = true;      // Start recording by default
volatile bool ads1Enabled = true;      // ADC 1 enabled by default
volatile bool ads2Enabled = true;      // ADC 2 enabled by default
volatile bool ads3Enabled = true;      // ADC 3 enabled by default
volatile bool ads4Enabled = true;      // ADC 4 enabled by default
volatile uint32_t buttonPressStartTime = 0;  // For tracking press duration
volatile uint32_t totalButtonPressTime = 0;  // For tracking cumulative press time
volatile bool buttonPressed = false;   // Button press state
volatile bool shouldReboot = false;    // Flag to trigger reboot
RGB* statusLED = nullptr;              // RGB LED for status indication

struct AdcTaskParams {
    const char* name;
    I2C* i2c;
    int startChannel;
};

struct sensorData {
    uint16_t channels[8];
};
sensorData sData;

uint8_t buffer[2];

// SD CARD FUNCTIONS
bool initSDCard() {
    printf("\n\nInitializing SD Card...\n");
    
    // Initialize SPI bus
    spiMaster = new SPI(SPI::SpiMode::kMaster, SPI2_HOST, kMosiPin, kMisoPin, kSclPin);
    esp_err_t err = spiMaster->init();
    if (err) {
        printf("SPI error while init: %s\n", esp_err_to_name(err));
        return false;
    }

    // Initialize SD card
    sdCard = new SD(spiMaster, kCsPin);
    err = sdCard->init();
    if (err) {
        printf("SD error while init: %s\n", esp_err_to_name(err));
        return false;
    } else {
        printf("SD was initialized!\n");
    }

    // Mount SD card
    FRESULT error = sdCard->mountCard();
    if (error) {
        printf("SD error while mounting: %s\n", sdCard->getFastFsErrName(error));
        return false;
    } else {
        std::string currentPath = sdCard->getCurrentDir();
        printf("Card mounted, root path: %s\n", currentPath.c_str());
    }

    // Print SD card info
    if (sdCard->sdCardInfo_.is_mem) {
        printf("SD is memory card.\n");
    }
    printf("Max freq speed (kHz): %d\n", (int)sdCard->sdCardInfo_.max_freq_khz);
    printf("Real freq speed (kHz): %d\n", sdCard->sdCardInfo_.real_freq_khz);
    
    return true;
}

bool createAndWriteFile(const std::string& dirPath, const std::string& fileName, const std::string& content) {
    if (!sdCard) {
        printf("SD card not initialized\n");
        return false;
    }
    
    // Create directory structure if it doesn't exist
    std::string currentDir = sdCard->getCurrentDir();
    FRESULT error;
    
    // Check if we need to create/navigate to a directory
    if (!dirPath.empty() && dirPath != "/") {
        // Create directory (this will fail if already exists, but that's OK)
        error = sdCard->createDir(dirPath);
        
        // Navigate to the directory
        error = sdCard->goToDir(sdCard->getCurrentDir() + "/" + dirPath);
        if (error) {
            printf("SD error at goToDir: %s\n", sdCard->getFastFsErrName(error));
            return false;
        } else {
            //printf("Directory changed, current path: %s\n", sdCard->getCurrentDir().c_str());
        }
    }
    
    // Create file
    error = sdCard->createFile(fileName);
    if (error && error != FR_EXIST) {  // Ignore if file already exists
        printf("SD error at createFile: %s\n", sdCard->getFastFsErrName(error));
        return false;
    }
    
    // Open file for writing
    error = sdCard->openFile(fileName, SD::openMode::kOpenAppend);
    if (error) {
        printf("SD error at openFile: %s\n", sdCard->getFastFsErrName(error));
        return false;
    }
    
    // Write to file
    //printf("Writing to file: '%s'\n", content.c_str());
    error = sdCard->fileWrite(content);
    if (error) {
        printf("SD error at fileWrite: %s\n", sdCard->getFastFsErrName(error));
        sdCard->closeFile();
        return false;
    }
    
    // Close file
    error = sdCard->closeFile();
    if (error) {
        printf("SD error at closeFile: %s\n", sdCard->getFastFsErrName(error));
        return false;
    }
    
    //printf("File write completed successfully!\n");
    
    // Return to original directory
    sdCard->goToDir(currentDir);
    
    return true;
}

std::string readFile(const std::string& dirPath, const std::string& fileName) {
    if (!sdCard) {
        printf("SD card not initialized\n");
        return "";
    }
    
    std::string fileContents;
    std::string currentDir = sdCard->getCurrentDir();
    FRESULT error;
    
    // Navigate to directory if needed
    if (!dirPath.empty() && dirPath != "/") {
        error = sdCard->goToDir(dirPath.substr(0, 1) == "/" ? dirPath : currentDir + "/" + dirPath);
        if (error) {
            printf("SD error at goToDir: %s\n", sdCard->getFastFsErrName(error));
            return "";
        }
    }
    
    // Open file for reading
    error = sdCard->openFile(fileName, SD::openMode::kOpenReadOnly);
    if (error) {
        printf("SD error at openFile: %s\n", sdCard->getFastFsErrName(error));
        sdCard->goToDir(currentDir);  // Return to original directory
        return "";
    }
    
    // Read file contents
    error = sdCard->fileRead(fileContents);
    if (error) {
        printf("SD error at fileRead: %s\n", sdCard->getFastFsErrName(error));
        sdCard->closeFile();
        sdCard->goToDir(currentDir);  // Return to original directory
        return "";
    }
    
    // Close file
    sdCard->closeFile();
    
    // Return to original directory
    sdCard->goToDir(currentDir);
    
    return fileContents;
}

void cleanupSDCard() {
    if (sdCard) {
        sdCard->unmountCard();
        printf("Card unmounted\n");
        delete sdCard;
        sdCard = nullptr;
    }
    
    if (spiMaster) {
        delete spiMaster;
        spiMaster = nullptr;
    }
}


// Forward declaration for buttonHandlerTask
void buttonHandlerTask(void* arg);

extern "C" void app_main() {
    // Initialize RGB LED for status indication
    statusLED = new RGB();
    esp_err_t err = statusLED->init();
    if (err) {
        printf("LED initialization error: %s\n", esp_err_to_name(err));
    } else {
        // Show boot animation
        statusLED->setColor(0, 0, 255); // Blue during boot
        statusLED->turnOn();
    }

    // Start button handler task
    xTaskCreate(buttonHandlerTask, "ButtonHandlerTask", 4096, NULL, 5, NULL);
    
    // Initialize I2C
    i2c0.init();
    i2c1.init();

    ADS1015 ads1(i2c0, ADS1015::ADS111X_Address::ADS111X_ADDR_GND);
    ADS1015 ads2(i2c0, ADS1015::ADS111X_Address::ADS111X_ADDR_VCC);
    ADS1015 ads3(i2c1, ADS1015::ADS111X_Address::ADS111X_ADDR_GND);
    ADS1015 ads4(i2c1, ADS1015::ADS111X_Address::ADS111X_ADDR_VCC);

    bool adsStatus = true;
    if (!ads1.checkForDevice()){
        printf("ADS1 not found\n");
        adsStatus = false;
    }
    if (!ads2.checkForDevice()){
        printf("ADS2 not found\n");
        adsStatus = false;
    }    if (!ads3.checkForDevice()){
        printf("ADS3 not found\n");
        adsStatus = false;
    }
    if (!ads4.checkForDevice()){
        printf("ADS4 not found\n");
        adsStatus = false;
    }

    // Initialize SD card
    bool sdInitialized = initSDCard();
    if (!sdInitialized) {
        printf("Failed to initialize SD card\n");
        // Indicate SD card error with LED
        if (statusLED) {
            statusLED->setColor(255, 0, 0); // Red for error
            statusLED->pulse(300);
        }
    }

    // If initialization is successful, show green LED
    if (sdInitialized && adsStatus) {
        if (statusLED) {
            statusLED->setColor(0, 255, 0); // Green for ready
            statusLED->turnOn();
        }
    } else if (statusLED) {
        // Something failed during initialization
        statusLED->setColor(255, 165, 0); // Orange for partial failure
        statusLED->turnOn();
    }

    // Create session directory with timestamp
    char sessionDir[32];
    snprintf(sessionDir, sizeof(sessionDir), "session_%lld", esp_timer_get_time() / 1000000);
    
    // Create header for data files
    std::string headerLine = "ch1_adc1,ch2_adc1,ch3_adc1,ch4_adc1,ch1_adc2,ch2_adc2,ch3_adc2,ch4_adc2,ch1_adc3,ch2_adc3,ch3_adc3,ch4_adc3,ch1_adc4,ch2_adc4,ch3_adc4,ch4_adc4\n";
    
    // Start ADC reading loop
    vTaskDelay(pdMS_TO_TICKS(3000));  // Initial delay to allow setup to complete
    int sampleCount = 0;
    uint64_t fileStartTime = esp_timer_get_time() / 1000000; // File start time in seconds
    uint64_t currentTime = fileStartTime;
    char currentFilename[64];
    snprintf(currentFilename, sizeof(currentFilename), "data_%lld.csv", fileStartTime);
    std::string adcDataBuffer;
    
    // Write header to file
    if (sdInitialized) {
        createAndWriteFile(sessionDir, currentFilename, headerLine);
    }
    
    while (true) {
        // Only collect and save data if recording is enabled
        if (isRecording) {
            adcDataBuffer.clear();
            
            for (int i = 0; i < 4; i++) {
                // Only read from enabled ADCs
                uint16_t value1 = ads1Enabled ? ads1.readSingleEndedSigned(i) : 0;
                uint16_t value2 = ads2Enabled ? ads2.readSingleEndedSigned(i) : 0;
                uint16_t value3 = ads3Enabled ? ads3.readSingleEndedSigned(i) : 0;
                uint16_t value4 = ads4Enabled ? ads4.readSingleEndedSigned(i) : 0;
                
                // Format data for SD card storage
                char dataLine[100];
                snprintf(dataLine, sizeof(dataLine), "%d,%d,%d,%d", value1, value2, value3, value4);
                
                // Add comma between sets but not after the last set
                if (i < 3) {
                    adcDataBuffer += dataLine;
                    adcDataBuffer += ",";
                } else {
                    adcDataBuffer += dataLine;
                }
            }
            adcDataBuffer += "\n";
            
            // Save data to SD card every few samples
            sampleCount++;
            if (sdInitialized && sampleCount >= 10) {
                createAndWriteFile(sessionDir, currentFilename, adcDataBuffer);
                sampleCount = 0;
                
                // Check if we need to create a new file (after 1 minute)
                currentTime = esp_timer_get_time() / 1000000;
                if (currentTime - fileStartTime >= 60) { // 60 seconds = 1 minute
                    fileStartTime = currentTime;
                    snprintf(currentFilename, sizeof(currentFilename), "data_%lld.csv", fileStartTime);
                    
                    // Write header to new file
                    createAndWriteFile(sessionDir, currentFilename, headerLine);
                    
                    printf("Created new data file: %s\n", currentFilename);
                }
                  // Print a sample of the data to console (for debugging)
                printf("%s", adcDataBuffer.c_str());
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay 10ms for a sampling rate of ~100Hz
    }
    
    // Cleanup (note: this won't be reached in this loop)
    cleanupSDCard();
    
    if (statusLED) {
        delete statusLED;
        statusLED = nullptr;
    }
}

// Button handling
void buttonCallback(void* arg) {
    if (!buttonPressed) {
        // Button just pressed
        buttonPressed = true;
        buttonPressStartTime = esp_timer_get_time() / 1000; // Convert to milliseconds
    } else {
        // Button released
        buttonPressed = false;
        uint32_t pressDuration = (esp_timer_get_time() / 1000) - buttonPressStartTime;
        totalButtonPressTime += pressDuration;
        
        // Handle different press durations
        if (pressDuration < 2000) {
            // Short press - toggle recording
            isRecording = !isRecording;
            if (isRecording) {
                printf("Recording started\n");
                if (statusLED) statusLED->setColor(0, 255, 0); // Green for recording
            } else {
                printf("Recording stopped\n");
                if (statusLED) statusLED->setColor(255, 0, 0); // Red for stopped
            }
        } else if (pressDuration >= 2000 && pressDuration < 5000) {
            // Medium press (2-5 seconds) - toggle ADC
            if (ads1Enabled && ads2Enabled && ads3Enabled && ads4Enabled) {
                ads1Enabled = false;
                printf("ADC 1 disabled\n");
                if (statusLED) statusLED->setColor(255, 165, 0); // Orange - 1 ADC disabled
            } else if (!ads1Enabled && ads2Enabled && ads3Enabled && ads4Enabled) {
                ads2Enabled = false;
                printf("ADC 2 disabled\n");
                if (statusLED) statusLED->setColor(255, 105, 180); // Pink - 2 ADCs disabled
            } else if (!ads1Enabled && !ads2Enabled && ads3Enabled && ads4Enabled) {
                ads3Enabled = false;
                printf("ADC 3 disabled\n");
                if (statusLED) statusLED->setColor(128, 0, 128); // Purple - 3 ADCs disabled
            } else if (!ads1Enabled && !ads2Enabled && !ads3Enabled && ads4Enabled) {
                ads4Enabled = false;
                printf("ADC 4 disabled\n");
                if (statusLED) statusLED->setColor(255, 255, 0); // Yellow - all ADCs disabled
            } else {
                // Re-enable all ADCs
                ads1Enabled = true;
                ads2Enabled = true;
                ads3Enabled = true;
                ads4Enabled = true;
                printf("All ADCs enabled\n");
                if (statusLED) statusLED->setColor(0, 255, 0); // Green - all enabled
            }
        }
        
        // Check for reboot condition (accumulated 5+ seconds of pressing)
        if (totalButtonPressTime >= 5000) {
            printf("Rebooting system...\n");
            if (statusLED) {
                // Flash white rapidly to indicate reboot
                for (int i = 0; i < 5; i++) {
                    statusLED->setColor(255, 255, 255);
                    statusLED->turnOn();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    statusLED->turnOff();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            shouldReboot = true;
        }
    }
}

// Button task handler
void buttonHandlerTask(void* arg) {
    // Initialize button with external pull-up
    Switch userButton(kButtonPin, Switch::SwitchMode::kNormallyOpen, true);
    
    // Configure interrupt for both rising and falling edges
    esp_err_t err = userButton.configureInterrupt(GPIO_INTR_ANYEDGE, buttonCallback, nullptr);
    if (err) {
        printf("Button interrupt configuration error: %s\n", esp_err_to_name(err));
    }
    
    // Start button handler task
    userButton.startHandlerTask("ButtonHandlerTask", 5, 2048);
    
    // Initialize button
    err = userButton.init();
    if (err) {
        printf("Button initialization error: %s\n", esp_err_to_name(err));
    }
    
    // Monitoring loop
    while (true) {
        // Check for reboot request
        if (shouldReboot) {
            printf("Rebooting...\n");
            esp_restart();
        }
        
        // Simple heartbeat for the button task
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}