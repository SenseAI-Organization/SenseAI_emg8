/*******************************************************************************
 * @file flash_sense.cpp
 * @brief Contains the classes and method definitions to work with the NVS
 * (flash memory).
 *
 * @version 0.2.3
 * @date 2024-08-29
 * @author emmanuel@sense-ai.co, Sense AI.
 *******************************************************************************
 *******************************************************************************/

#pragma once

#include <esp_err.h>

#include <string>

#include "nvs.h"
#include "nvs_flash.h"

#ifndef CONFIG_NVS_ENCRYPTION
#warning \
    "NVS Encryption is not enabled. Enable it if needed. \
    To enable it, set CONFIG_NVS_ENCRYPTION=1 in menuconfig -> Component config -> NVS."
#endif

/**
 * @class FlashStorage
 * @brief Manages non-volatile storage (NVS) on ESP32 devices.
 *
 * This class encapsulates operations for reading, writing, and managing
 * data in NVS, offering a simple interface for handling persistent storage.
 */
class FlashStorage {
public:
    /**
     * @brief Constructor that initializes the FlashStorage with a namespace.
     * @param namespaceName Name of the NVS namespace used for storage.
     * @param encrypted Enable NVS encryption (default: false). Requires
     * CONFIG_NVS_ENCRYPTION=1.
     * @param partitionLabel Partition label for encryption keys (default: "nvs_key").
     */
#ifdef CONFIG_NVS_ENCRYPTION
    FlashStorage(const char* namespaceName, bool encrypted = false,
                 const char* partitionLabel = "nvs_key");
#else
    FlashStorage(const char* namespaceName);
#endif

    /**
     * @brief Destructor that ensures NVS is properly closed if still open.
     */
    ~FlashStorage();

    /**
     * @brief Initializes the NVS storage area.
     * @return esp_err_t ESP_OK on success, error code on failure.
     */
    esp_err_t init(void);

    /**
     * @brief Retrieves an unsigned 8-bit integer from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved value.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, uint8_t& value);

    /**
     * @brief Retrieves a signed 8-bit integer from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved value.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, int8_t& value);

    /**
     * @brief Retrieves an unsigned 16-bit integer from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved value.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, uint16_t& value);

    /**
     * @brief Retrieves a signed 16-bit integer from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved value.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, int16_t& value);

    /**
     * @brief Retrieves an unsigned 32-bit integer from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved value.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, uint32_t& value);

    /**
     * @brief Retrieves a signed 32-bit integer from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved value.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, int32_t& value);

    /**
     * @brief Retrieves a string from NVS.
     * @param key Key under which the value is stored.
     * @param value Reference to store the retrieved string.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, std::string& value);

    /**
     * @brief Retrieves a binary blob from NVS.
     * @param key Key under which the value is stored.
     * @param buffer Pointer to the buffer to store the blob.
     * @param length Reference to the size of the buffer. On success,
     *               it will contain the actual size of the blob.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t get(const char* key, void* buffer, size_t& length);

    /**
     * @brief Stores an unsigned 8-bit integer in NVS.
     * @param key Key under which the value is to be stored.
     * @param value Value to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, uint8_t value);

    /**
     * @brief Stores a signed 8-bit integer in NVS.
     * @param key Key under which the value is to be stored.
     * @param value Value to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, int8_t value);

    /**
     * @brief Stores an unsigned 16-bit integer in NVS.
     * @param key Key under which the value is to be stored.
     * @param value Value to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, uint16_t value);

    /**
     * @brief Stores a signed 16-bit integer in NVS.
     * @param key Key under which the value is to be stored.
     * @param value Value to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, int16_t value);

    /**
     * @brief Stores an unsigned 32-bit integer in NVS.
     * @param key Key under which the value is to be stored.
     * @param value Value to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, uint32_t value);

    /**
     * @brief Stores a signed 32-bit integer in NVS.
     * @param key Key under which the value is to be stored.
     * @param value Value to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, int32_t value);

    /**
     * @brief Stores a string in NVS.
     * @param key Key under which the value is to be stored.
     * @param value String to be stored.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, const std::string& value);

    /**
     * @brief Stores a binary blob in NVS.
     * @param key Key under which the value is to be stored.
     * @param buffer Pointer to the buffer containing the blob.
     * @param length Size of the blob.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t put(const char* key, const void* buffer, size_t length);

    /**
     * @brief Commits changes to NVS, ensuring data persistence.
     */
    esp_err_t commit(void);

    /**
     * @brief Removes a key and its associated value from NVS.
     * @param key Key to be removed.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t remove(const char* key);

    /**
     * @brief Erases all keys and values in the current namespace.
     * @return `ESP_OK` on success, or an error code if the operation fails.
     */
    esp_err_t reset(void);

    /**
     * @brief Closes the NVS handle, optionally committing changes first.
     */
    void end(void);

    /**
     * @brief Returns true if the flash is initialized, false otherwise.
     */
    bool isInitialized(void) const;

private:
    char* namespaceName_;     ///< Namespace for NVS operations.
    nvs_handle_t nvsHandle_;  ///< Handle for NVS access.
    bool initialized_;        ///< Indicates if NVS has been initialized.
#ifdef CONFIG_NVS_ENCRYPTION
    bool encrypted_;              ///< Indicates if NVS encryption is enabled.
    const char* partitionLabel_;  ///< Partition label for encryption keys.
#endif
};
