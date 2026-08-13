/*******************************************************************************
 * @file flash_encrypted_example.cpp
 * @brief Example demonstrating encrypted NVS flash storage usage.
 *
 * This example shows how to use the FlashStorage class with encryption enabled.
 *
 * Requirements:
 * - CONFIG_NVS_ENCRYPTION=1 must be set in menuconfig
 * - Partition table must include 'nvs_key' partition for encryption keys
 * - CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC=y recommended for production
 *
 * @version 0.2.4
 * @date 2026-02-03
 * @author Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "flash_sense.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main() {
    printf("Starting Encrypted Flash Storage Example\n");

#ifdef CONFIG_NVS_ENCRYPTION
    // Create encrypted flash storage instance
    // Parameters: namespace, encrypted (true), partition_label ("nvs_key")
    FlashStorage secureFlash("secrets", true, "nvs_key");

    esp_err_t err = secureFlash.init();
    if (err != ESP_OK) {
        printf("Failed to initialize encrypted flash: %s\n", esp_err_to_name(err));
        return;
    }
    printf("Encrypted flash initialized successfully\n");

    // Store sensitive data - all data will be encrypted in flash
    printf("Storing encrypted credentials...\n");

    // Store a password
    std::string password = "MySecurePassword123!";
    err = secureFlash.put("wifi_pass", password);
    if (err != ESP_OK) {
        printf("Failed to save password: %s\n", esp_err_to_name(err));
    } else {
        printf("Password saved securely\n");
    }

    // Store an API key
    std::string apiKey = "sk-1234567890abcdef";
    err = secureFlash.put("api_key", apiKey);
    if (err != ESP_OK) {
        printf("Failed to save API key: %s\n", esp_err_to_name(err));
    } else {
        printf("API key saved securely\n");
    }

    // Store numeric values (also encrypted)
    uint32_t deviceId = 0xDEADBEEF;
    err = secureFlash.put("device_id", deviceId);
    if (err != ESP_OK) {
        printf("Failed to save device ID: %s\n", esp_err_to_name(err));
    } else {
        printf("Device ID saved: 0x%lX\n", deviceId);
    }

    // Commit changes to persist data
    err = secureFlash.commit();
    if (err != ESP_OK) {
        printf("Failed to commit changes: %s\n", esp_err_to_name(err));
        return;
    }
    printf("Changes committed successfully\n");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Read back encrypted data
    printf("\nReading encrypted data...\n");

    std::string loadedPassword;
    err = secureFlash.get("wifi_pass", loadedPassword);
    if (err == ESP_OK) {
        printf("Password loaded: %s\n", loadedPassword.c_str());
    } else {
        printf("Failed to load password: %s\n", esp_err_to_name(err));
    }

    std::string loadedApiKey;
    err = secureFlash.get("api_key", loadedApiKey);
    if (err == ESP_OK) {
        printf("API key loaded: %s\n", loadedApiKey.c_str());
    } else {
        printf("Failed to load API key: %s\n", esp_err_to_name(err));
    }

    uint32_t loadedDeviceId = 0;
    err = secureFlash.get("device_id", loadedDeviceId);
    if (err == ESP_OK) {
        printf("Device ID loaded: 0x%lX\n", loadedDeviceId);
    } else {
        printf("Failed to load device ID: %s\n", esp_err_to_name(err));
    }

    // Demonstrate deletion of sensitive data
    printf("\nDeleting API key...\n");
    err = secureFlash.remove("api_key");
    if (err == ESP_OK) {
        printf("API key deleted successfully\n");
    } else {
        printf("Failed to delete API key: %s\n", esp_err_to_name(err));
    }

    err = secureFlash.commit();
    if (err != ESP_OK) {
        printf("Failed to commit deletion: %s\n", esp_err_to_name(err));
    }

    // Close the flash storage
    secureFlash.end();
    printf("Encrypted flash storage closed\n");

    // Example: Using multiple encrypted namespaces
    printf("\nUsing multiple encrypted namespaces...\n");

    FlashStorage userConfig("user_cfg", true);
    FlashStorage deviceConfig("dev_cfg", true);

    if (userConfig.init() == ESP_OK) {
        userConfig.put("username", std::string("admin"));
        userConfig.put("email", std::string("admin@example.com"));
        userConfig.commit();
        printf("User config saved in separate encrypted namespace\n");
        userConfig.end();
    }

    if (deviceConfig.init() == ESP_OK) {
        deviceConfig.put("serial_number", uint32_t(123456));
        deviceConfig.put("firmware_ver", std::string("v1.0.0"));
        deviceConfig.commit();
        printf("Device config saved in separate encrypted namespace\n");
        deviceConfig.end();
    }

    printf("\nEncrypted flash example completed successfully!\n");

#else
    printf("CONFIG_NVS_ENCRYPTION is not enabled!\n");
    printf("Add '-D CONFIG_NVS_ENCRYPTION=1' to build_flags in platformio.ini\n");
    printf("And ensure nvs_key partition exists in partition table\n");
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
