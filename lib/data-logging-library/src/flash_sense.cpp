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

#include "esp_log.h"
#include "flash_sense.hpp"

#ifdef CONFIG_NVS_ENCRYPTION
#include "esp_partition.h"

FlashStorage::FlashStorage(const char* ns, bool encrypted, const char* partitionLabel)
    : namespaceName_((char*)ns),
      nvsHandle_(0),
      initialized_(false),
      encrypted_(encrypted),
      partitionLabel_(partitionLabel) {
}
#else
FlashStorage::FlashStorage(const char* ns)
    : namespaceName_((char*)ns), nvsHandle_(0), initialized_(false) {
}
#endif

FlashStorage::~FlashStorage() {
    commit();
    end();
}

esp_err_t FlashStorage::init(void) {
    esp_err_t err;

#ifdef CONFIG_NVS_ENCRYPTION
    if (encrypted_) {
        // Initialize with encryption
        // Find the NVS keys partition
        const esp_partition_t* key_partition = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS,
            partitionLabel_);

        if (key_partition == NULL) {
            ESP_LOGE("FlashStorage", "NVS key partition '%s' not found", partitionLabel_);
            return ESP_ERR_NVS_PART_NOT_FOUND;
        }

        nvs_sec_cfg_t cfg;
        err = nvs_flash_read_security_cfg(key_partition, &cfg);

        if (err == ESP_ERR_NVS_KEYS_NOT_INITIALIZED || err == ESP_ERR_NOT_FOUND ||
            err == ESP_ERR_NVS_CORRUPT_KEY_PART) {
            // Erase corrupted partition or generate new encryption keys
            if (err == ESP_ERR_NVS_CORRUPT_KEY_PART) {
                ESP_LOGW("FlashStorage", "NVS key partition corrupted, erasing...");
                err = esp_partition_erase_range(key_partition, 0, key_partition->size);
                if (err != ESP_OK) {
                    ESP_LOGE("FlashStorage", "Failed to erase key partition: %s",
                             esp_err_to_name(err));
                    return err;
                }
            }

            // Generate new encryption keys
            err = nvs_flash_generate_keys(key_partition, &cfg);
            if (err != ESP_OK) {
                ESP_LOGE("FlashStorage", "Failed to generate keys: %s",
                         esp_err_to_name(err));
                return err;
            }
        } else if (err != ESP_OK) {
            ESP_LOGE("FlashStorage", "Failed to read security config: %s",
                     esp_err_to_name(err));
            return err;
        }

        // Initialize encrypted NVS
        err = nvs_flash_secure_init(&cfg);

        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            err = nvs_flash_erase();
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_flash_secure_init(&cfg);
        }
    } else
#endif
    {
        // Initialize without encryption
        err = nvs_flash_init();

        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // This can happen when the NVS partition schema is changed.
            err = nvs_flash_erase();

            if (err != ESP_OK) {
                return err;
            }

            err = nvs_flash_init();  // Try initializing again after erasing
        }
    }

    if (err != ESP_OK) {
        return err;
    }

    err = nvs_open(namespaceName_, NVS_READWRITE, &nvsHandle_);

    if (err != ESP_OK) {
        return err;
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t FlashStorage::get(const char* key, uint8_t& value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_get_u8(nvsHandle_, key, &value);
}

esp_err_t FlashStorage::get(const char* key, int8_t& value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_get_i8(nvsHandle_, key, &value);
}

esp_err_t FlashStorage::get(const char* key, uint16_t& value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_get_u16(nvsHandle_, key, &value);
}

esp_err_t FlashStorage::get(const char* key, int16_t& value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_get_i16(nvsHandle_, key, &value);
}

esp_err_t FlashStorage::get(const char* key, uint32_t& value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_get_u32(nvsHandle_, key, &value);
}

esp_err_t FlashStorage::get(const char* key, int32_t& value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_get_i32(nvsHandle_, key, &value);
}

esp_err_t FlashStorage::get(const char* key, std::string& value) {
    size_t required_size = 0;
    esp_err_t err = nvs_get_str(nvsHandle_, key, nullptr, &required_size);
    if (err != ESP_OK) {
        return err;
    }

    char* buffer = new char[required_size];
    err = nvs_get_str(nvsHandle_, key, buffer, &required_size);
    if (err == ESP_OK) {
        value.assign(buffer);
    }
    delete[] buffer;
    return err;
}

esp_err_t FlashStorage::get(const char* key, void* buffer, size_t& length) {
    return nvs_get_blob(nvsHandle_, key, buffer, &length);
}

esp_err_t FlashStorage::put(const char* key, uint8_t value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_set_u8(nvsHandle_, key, value);
}

esp_err_t FlashStorage::put(const char* key, int8_t value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_set_i8(nvsHandle_, key, value);
}

esp_err_t FlashStorage::put(const char* key, uint16_t value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_set_u16(nvsHandle_, key, value);
}

esp_err_t FlashStorage::put(const char* key, int16_t value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_set_i16(nvsHandle_, key, value);
}

esp_err_t FlashStorage::put(const char* key, uint32_t value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_set_u32(nvsHandle_, key, value);
}

esp_err_t FlashStorage::put(const char* key, int32_t value) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }
    return nvs_set_i32(nvsHandle_, key, value);
}

esp_err_t FlashStorage::put(const char* key, const std::string& value) {
    return nvs_set_str(nvsHandle_, key, value.c_str());
}

esp_err_t FlashStorage::put(const char* key, const void* buffer, size_t length) {
    return nvs_set_blob(nvsHandle_, key, buffer, length);
}

esp_err_t FlashStorage::commit(void) {
    if (!initialized_) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    return nvs_commit(nvsHandle_);
}

esp_err_t FlashStorage::remove(const char* key) {
    if (!initialized_) {
        ESP_LOGE("FlashStorage", "NVS not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_erase_key(nvsHandle_, key);
    if (err != ESP_OK) {
        ESP_LOGE("FlashStorage", "Failed to remove key '%s': %s", key,
                 esp_err_to_name(err));
    } else {
        ESP_LOGI("FlashStorage", "Key '%s' removed successfully", key);
    }
    return err;
}

esp_err_t FlashStorage::reset(void) {
    if (!initialized_) {
        ESP_LOGE("FlashStorage", "NVS not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = nvs_erase_all(nvsHandle_);
    if (err != ESP_OK) {
        ESP_LOGE("FlashStorage", "Failed to reset namespace: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("FlashStorage", "Namespace reset successfully");
    }
    return err;
}

void FlashStorage::end(void) {
    if (initialized_) {
        nvs_close(nvsHandle_);
        initialized_ = false;
    }
}

bool FlashStorage::isInitialized(void) const {
    return initialized_;
}
