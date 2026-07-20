/*******************************************************************************
 * @file sd_by_spi_tester.cpp
 * @brief Contains the test example for using a micro SD via SPI.
 *
 * @version 0.1.1
 * @date 2025-08-21
 * @author mateor@sense-ai.co - Sense AI
 *******************************************************************************
 *******************************************************************************/

#include "sd_storage_sense.hpp"
#include "spi_gp_sense.hpp"

constexpr gpio_num_t kCsPin = GPIO_NUM_39;
constexpr gpio_num_t kSclPin = GPIO_NUM_12;
constexpr gpio_num_t kMosiPin = GPIO_NUM_11;
constexpr gpio_num_t kMisoPin = GPIO_NUM_13;

extern "C" void app_main() {
    printf("\n\n");
    // 1. Initialize SPI bus and SD driver
    SPI spiMaster(SPI::SpiMode::kMaster, SPI2_HOST, kMosiPin, kMisoPin, kSclPin);
    // To initialize spi bus
    esp_err_t err = spiMaster.init();
    if (err) {
        printf("SPI error while init: %s\n", esp_err_to_name(err));
    }

    SD sdCard(spiMaster, kCsPin);
    err = sdCard.init();

    if (err) {
        printf("SD error while init: %s\n", esp_err_to_name(err));
    } else {
        printf("SD was initialized!\n");
    }

    // 2. Mount SD card
    FRESULT error = sdCard.mountCard();
    if (err) {
        printf("SD error while mounting: %s\n", sdCard.getFastFsErrName(error));
    } else {
        std::string currentPath = sdCard.getCurrentDir();
        printf(("Card mounted, here is the root path - " + currentPath).c_str());
        printf("\n");
    }

    std::string dirName = "myDir";
    std::string subDirName = "mySubDir";
    std::string fileName = "file.txt";

    // 3. Create a directory
    error = sdCard.createDir(dirName);
    if (error) {
        printf("SD error at createDir: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("Directory created, current path - %s\n", sdCard.getCurrentDir().c_str());
    }

    // 4. Navigate to the new directory
    error = sdCard.goToDir(sdCard.getCurrentDir() + "/" + dirName);
    if (error) {
        printf("SD error at goToDir: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("Dir changed, current path - %s\n", sdCard.getCurrentDir().c_str());
        printf("\n");
    }

    // 4.1. Create a sub-directory
    error = sdCard.createDir(subDirName);
    if (error) {
        printf("SD error at createDir: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("Directory created, current path - %s\n", sdCard.getCurrentDir().c_str());
    }

    // 4.2. Navigate to the new directory
    error = sdCard.goToDir(sdCard.getCurrentDir() + "/" + subDirName);
    if (error) {
        printf("SD error at goToDir: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("Dir changed, current path - %s\n", sdCard.getCurrentDir().c_str());
        printf("\n");
    }

    // 5. Create and write to file
    error = sdCard.createFile(fileName);
    if (error) {
        printf("SD error at createFile: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("File created at: %s\n", sdCard.getCurrentDir().c_str());
    }

    error = sdCard.openFile(fileName, SD::OpenMode::kOpenAppend);
    if (error) {
        printf("SD error at openFile: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("File successfully opened at - %s\n", sdCard.getCurrentDir().c_str());
        printf("Trying to write: 'Life is more like fight than dance!'\n");

        error = sdCard.fileWrite("Life is more like fight than dance!\n");
        if (error) {
            printf("SD error at fileWrite: %s\n", sdCard.getFastFsErrName(error));
        }

        sdCard.closeFile();
        if (error) {
            printf("SD error at createFile: %s\n", sdCard.getFastFsErrName(error));
        }
        printf("Writting process done!\n");
    }

    // // 6. Read from file
    std::string fileContents;
    if (sdCard.openFile(fileName, SD::OpenMode::kOpenReadOnly) == FR_OK) {
        error = sdCard.fileRead(fileContents);
        if (error) {
            printf("Error while reading\n");
        }
        sdCard.closeFile();
        printf("File content:\n%s\n", fileContents.c_str());
    }

    // // 7. List directory contents
    SD::Index_t index = {};
    if (sdCard.listDir("0:/myDir", &index) == FR_OK) {
        printf("In directory '%s':\n", index.currentDirPath.c_str());
        printf("Files: %d, Dirs: %d\n", index.cntOfFiles, index.cntOfDirectories);
    }

    if (sdCard.listDir("0:/myDir/mySubDir", &index) == FR_OK) {
        printf("In directory '%s':\n", index.currentDirPath.c_str());
        printf("Files: %d, Dirs: %d\n", index.cntOfFiles, index.cntOfDirectories);
    }

    // 7.1. Test rename functionality
    printf("\n--- Testing rename functionality ---\n");

    // Rename the file
    error = sdCard.renameObj("file.txt", "renamed_file.txt");
    if (error) {
        printf("SD error at renameObj (file): %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("File successfully renamed from 'file.txt' to 'renamed_file.txt'\n");
    }

    // Navigate back to parent directory and rename the subdirectory
    sdCard.goToDir("0:/myDir");
    error = sdCard.renameObj("mySubDir", "renamedSubDir");
    if (error) {
        printf("SD error at renameObj (directory): %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("Directory successfully renamed from 'mySubDir' to 'renamedSubDir'\n");
    }

    // 8. Unmount card
    sdCard.unmountCard();
    if (error) {
        printf("SD error at unmountCard: %s\n", sdCard.getFastFsErrName(error));
    } else {
        printf("Card successfully unmounted\n");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
