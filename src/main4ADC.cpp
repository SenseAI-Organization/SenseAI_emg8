/*******************************************************************************
 * main4ADC.cpp — EMG8 Bracelet Firmware
 *
 * 4× ADS1015 mixed-rate continuous sampling with ALERT/RDY interrupts:
 *   ch 0, 1 = fast  (raw EMG   ≈ 1150 Hz per channel in All mode)
 *   ch 2, 3 = slow  (envelope  ≈   57 Hz per channel in All mode)
 *
 * ICM-42605 6-axis IMU at 200 Hz via SPI
 *
 * Binary SD logging  ·  UART CSV output  ·  Reed switch + serial commands
 *
 * SD binary format (per file):
 *   [20-byte header] "EMG8" | ver(3) | nADC(4) | nCh(4) | div |
 *                    epoch_s(u32) | bat_mV(u16) | bat_%(u8) | state(u8) |
 *                    imuODR(u8) | rsvd(3)
 *   [N × 8-byte EMG records]  type=0x00 | adc(u8) | ch(u8) | pad(u8) | ts(u16ms) | value(i16)
 *   [N × 20-byte IMU records] type=0x01 | rsvd(u8) | ts(u16ms) |
 *                              ax(i16) | ay(i16) | az(i16) | gx(i16) | gy(i16) | gz(i16) |
 *                              temp_x100(i16)
 *
 * Sense-AI
 ******************************************************************************/
#include <cstring>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "ADS1015.hpp"
#include "ICM42605.hpp"
#include "actuators_sense.hpp"
#include "switch_sense.hpp"
#include "sd_storage_sense.hpp"
#include "spi_gp_sense.hpp"
#include "BatteryManager.hpp"

/* ── Pin Map ───────────────────────────────────────────────────────────────── */

// I2C Bus 0 → ADC1 (ADDR_GND), ADC2 (ADDR_VCC)
static constexpr gpio_num_t kSDA0 = GPIO_NUM_6;
static constexpr gpio_num_t kSCL0 = GPIO_NUM_7;

// I2C Bus 1 → ADC3 (ADDR_GND), ADC4 (ADDR_VCC)
static constexpr gpio_num_t kSDA1 = GPIO_NUM_45;
static constexpr gpio_num_t kSCL1 = GPIO_NUM_47;

// ALERT / RDY per ADC
static constexpr gpio_num_t kRDY[] = {
    GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_15
};

// SD Card (SPI2)
static constexpr gpio_num_t kSD_CS   = GPIO_NUM_10;
static constexpr gpio_num_t kSD_MOSI = GPIO_NUM_11;
static constexpr gpio_num_t kSD_SCK  = GPIO_NUM_12;
static constexpr gpio_num_t kSD_MISO = GPIO_NUM_13;

// Peripherals
static constexpr gpio_num_t kLED  = GPIO_NUM_2;   // WS2812 RGB
static constexpr gpio_num_t kREED = GPIO_NUM_9;    // Reed switch

// IMU SPI (SPI3 / VSPI)
static constexpr gpio_num_t kIMU_MOSI = GPIO_NUM_35;
static constexpr gpio_num_t kIMU_SCK  = GPIO_NUM_36;
static constexpr gpio_num_t kIMU_MISO = GPIO_NUM_37;
static constexpr gpio_num_t kIMU_CS   = GPIO_NUM_38;
static constexpr gpio_num_t kIMU_INT  = GPIO_NUM_16;

// Battery / Power (Carguita 5V)
static constexpr gpio_num_t kBAT_ADC  = GPIO_NUM_8;   // %Bat voltage divider
static constexpr gpio_num_t k5V_EN    = GPIO_NUM_14;  // 5V boost enable
static constexpr gpio_num_t kCHG      = GPIO_NUM_5;   // Charge status (active LOW)
static constexpr gpio_num_t kPGOOD    = GPIO_NUM_4;   // Power-good (active LOW)

/* ── Channel layout (identical on every ADC) ───────────────────────────────── */

static constexpr uint8_t kEMG0     = 0;   // fast — raw EMG
static constexpr uint8_t kEMG1     = 1;   // fast — raw EMG
static constexpr uint8_t kENV0     = 2;   // slow — envelope
static constexpr uint8_t kENV1     = 3;   // slow — envelope
static constexpr uint8_t kSLOW_DIV = 20;  // ≈ 57 Hz envelope at 2400 SPS w/ 2 fast ch
static constexpr auto    kADC_RATE = ADS1015::ConfigRate::Rate_2400Hz;

static constexpr uint16_t kIMU_ODR_HZ = 200;      // IMU polling rate

/* ── Types ─────────────────────────────────────────────────────────────────── */

struct __attribute__((packed)) Sample {
    uint32_t ts;    // µs since recording start
    uint8_t  adc;   // 0–3
    uint8_t  ch;    // 0–3
    int16_t  val;   // signed 12-bit
};
static_assert(sizeof(Sample) == 8, "Sample must be 8 bytes");

/** @brief IMU sample for SD logging (20 bytes, raw int16 for compactness). */
struct __attribute__((packed)) ImuSample {
    uint32_t ts;       // µs since recording start
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int16_t  temp100;  // temperature × 100
    uint16_t _pad;     // pad to 20 bytes
};
static_assert(sizeof(ImuSample) == 20, "ImuSample must be 20 bytes");

enum class Mode : uint8_t { Idle = 0, All = 1, Raw = 2, Env = 3 };

/* ── Globals ───────────────────────────────────────────────────────────────── */

static I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 1000000, false);
static I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 1000000, false);

static ADS1015* adc[4];
static SPI*     spiSD  = nullptr;
static SPI*     spiIMU = nullptr;
static SD*      sdCard = nullptr;
static RGB*     led    = nullptr;
static Switch*  reedSw = nullptr;
static BatteryManager* battery = nullptr;
static ICM42605*       imu     = nullptr;
static bool            imuOK   = false;

static volatile Mode mode      = Mode::Idle;
static volatile bool recording = false;
static int64_t       recStart  = 0;            // µs epoch for timestamps
static bool          sdOK      = false;        // SD card available

static QueueHandle_t sdQ  = nullptr;
static QueueHandle_t imuQ = nullptr;
static constexpr int kQLEN    = 8000;          // ≈ 600 ms buffer at 13.2 kSa/s
static constexpr int kIMU_QLEN = 400;          // ≈ 2 s buffer at 200 Hz

/* ── ADC conversion callback (called from each ADC's FreeRTOS task) ──────── */

static void onSample(uint8_t ch, int16_t val, void* arg) {
    uint8_t id = (uint8_t)(uintptr_t)arg;
    bool fast  = (ch <= kEMG1);

    // Drop channels the current mode doesn't need
    if (mode == Mode::Raw && !fast) return;
    if (mode == Mode::Env &&  fast) return;

    Sample s;
    s.ts  = (uint32_t)(esp_timer_get_time() - recStart);
    s.adc = id;
    s.ch  = ch;
    s.val = val;
    xQueueSend(sdQ, &s, 0);   // non-blocking; drop if full
}

/* ── SD writer task — pinned to core 1, priority 5 ────────────────────────── */

static void sdWriteTask(void*) {
    constexpr int BATCH = 500;  // ~4 KB per write
    constexpr int IMU_BATCH = 50;
    Sample buf[BATCH];
    ImuSample imuBuf[IMU_BATCH];
    char   fname[32];
    bool   fOpen   = false;
    int64_t fEpoch = 0;

    // Session directory
    char sdir[24];
    snprintf(sdir, sizeof(sdir), "s_%lld",
             (long long)(esp_timer_get_time() / 1000000));
    sdCard->createDir(std::string(sdir));
    sdCard->goToDir(std::string(sdir));

    while (true) {
        /* ---- idle when not recording ---- */
        if (!recording) {
            if (fOpen) { sdCard->closeFile(); fOpen = false; }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ---- open / rotate file ---- */
        int64_t nowSec = esp_timer_get_time() / 1000000;
        if (!fOpen) {
            fEpoch = nowSec;
            snprintf(fname, sizeof(fname), "emg_%lld.bin", (long long)fEpoch);
            sdCard->createFile(std::string(fname));
            sdCard->openFile(std::string(fname), SD::OpenMode::kOpenAppend);

            // 20-byte header  (v3 — adds IMU ODR field)
            // [0-3] "EMG8"  [4] ver  [5] nADC  [6] nCh  [7] div
            // [8-11] epoch_s(u32)  [12-13] bat_mV(u16)  [14] bat_%
            // [15] bat_state  [16] imuODR(u8)  [17-19] reserved
            uint8_t hdr[20] = {};
            memcpy(hdr, "EMG8", 4);
            hdr[4] = 3;           // version
            hdr[5] = 4;           // number of ADCs
            hdr[6] = 4;           // channels per ADC
            hdr[7] = kSLOW_DIV;   // slow divider
            uint32_t e32 = (uint32_t)fEpoch;
            memcpy(hdr + 8, &e32, 4);
            if (battery) {
                battery->measure();
                uint16_t mv = battery->getVoltage();
                memcpy(hdr + 12, &mv, 2);
                hdr[14] = battery->getPercentage();
                hdr[15] = (uint8_t)battery->getState();
            }
            hdr[16] = (uint8_t)kIMU_ODR_HZ;  // 200
            sdCard->fileWrite(std::string(reinterpret_cast<char*>(hdr), 20));
            fOpen = true;
        } else if (nowSec - fEpoch >= 60) {
            sdCard->closeFile();
            fOpen = false;
            continue;   // will reopen on next iteration
        }

        /* ---- drain EMG queue → batch write ---- */
        int n = 0;
        while (n < BATCH && xQueueReceive(sdQ, &buf[n], 0) == pdTRUE) n++;
        if (n > 0) {
            // Tag byte 0xAA before EMG block (1 + n*8 bytes)
            uint8_t tag = 0xAA;
            sdCard->fileWrite(std::string(reinterpret_cast<char*>(&tag), 1));
            sdCard->fileWrite(
                std::string(reinterpret_cast<char*>(buf), n * sizeof(Sample)));
        }

        /* ---- drain IMU queue → batch write ---- */
        int nImu = 0;
        while (nImu < IMU_BATCH && xQueueReceive(imuQ, &imuBuf[nImu], 0) == pdTRUE) nImu++;
        if (nImu > 0) {
            // Tag byte 0xBB before IMU block (1 + nImu*20 bytes)
            uint8_t tag = 0xBB;
            sdCard->fileWrite(std::string(reinterpret_cast<char*>(&tag), 1));
            sdCard->fileWrite(
                std::string(reinterpret_cast<char*>(imuBuf), nImu * sizeof(ImuSample)));
        }

        if (n == 0 && nImu == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));  // yield when idle
        }
    }
}

/* ── UART output task — pinned to core 0, ~50 Hz ──────────────────────────── */
/*  Protocol: line-based text, prefixed by a tag character.
 *
 *  Outgoing (ESP → PC):
 *    '#' lines = metadata / status    e.g.  #READY  #MODE:1  #CD:3  #REC  #STOP
 *    'H' line  = CSV column header    e.g.  H,adc1_0,adc1_1,...,ax,ay,az,gx,gy,gz
 *    'D' line  = CSV data             e.g.  D,1234,-456,...,0.12,-0.03,0.98,1.2,0.5,-0.1
 *
 *  Incoming (PC → ESP):
 *    '0'  = stop recording
 *    '1'  = start/switch to All mode
 *    '2'  = start/switch to Raw mode
 *    '3'  = start/switch to Env mode
 *    '?'  = query status  → responds with #STATUS:<mode>,<recording>,<bat_mV>,<bat_%>
 */

static void uartTask(void*) {
    bool hdrDone = false;
    Mode lastHdrMode = Mode::Idle;

    while (true) {
        if (!recording) {
            hdrDone = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Reprint header when mode changes mid-recording
        if (mode != lastHdrMode) hdrDone = false;

        // Print header line once per recording start / mode change
        if (!hdrDone) {
            printf("H");
            for (int a = 0; a < 4; a++)
                for (int c = 0; c < 4; c++) {
                    bool fast = (c <= kEMG1);
                    if (mode == Mode::Raw && !fast) continue;
                    if (mode == Mode::Env &&  fast) continue;
                    printf(",adc%d_%d", a + 1, c);
                }
            if (imuOK)
                printf(",ax,ay,az,gx,gy,gz");
            printf("\n");
            lastHdrMode = mode;
            hdrDone = true;
        }

        // One CSV line of latest readings
        char line[384];
        int  p = 0;
        line[p++] = 'D';
        for (int a = 0; a < 4; a++)
            for (int c = 0; c < 4; c++) {
                bool fast = (c <= kEMG1);
                if (mode == Mode::Raw && !fast) continue;
                if (mode == Mode::Env &&  fast) continue;
                line[p++] = ',';
                p += snprintf(line + p, sizeof(line) - p, "%d",
                              adc[a]->getLatestReading(c));
            }
        // Append IMU (latest measurement already done by imuTask)
        if (imuOK) {
            const float* ac = imu->getAccel();
            const float* gy = imu->getGyro();
            p += snprintf(line + p, sizeof(line) - p,
                          ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                          ac[0], ac[1], ac[2], gy[0], gy[1], gy[2]);
        }
        line[p++] = '\n';
        line[p] = '\0';
        printf("%s", line);

        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz
    }
}

/* ── IMU polling task — pinned to core 1, 200 Hz ──────────────────────────── */

static void imuTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / kIMU_ODR_HZ);  // 5 ms
    TickType_t wake = xTaskGetTickCount();

    while (true) {
        if (!recording) {
            vTaskDelay(pdMS_TO_TICKS(100));
            wake = xTaskGetTickCount();
            continue;
        }

        if (imu->measure() == ESP_OK) {
            const float* ac = imu->getAccel();
            const float* gy = imu->getGyro();
            float temp = imu->getTemperature();

            ImuSample s;
            s.ts = (uint32_t)(esp_timer_get_time() - recStart);
            // Store as raw int16 scaled: accel in milli-g, gyro in deci-dps
            s.ax = (int16_t)(ac[0] * 1000.0f);
            s.ay = (int16_t)(ac[1] * 1000.0f);
            s.az = (int16_t)(ac[2] * 1000.0f);
            s.gx = (int16_t)(gy[0] * 10.0f);
            s.gy = (int16_t)(gy[1] * 10.0f);
            s.gz = (int16_t)(gy[2] * 10.0f);
            s.temp100 = (int16_t)(temp * 100.0f);
            xQueueSend(imuQ, &s, 0);
        }

        vTaskDelayUntil(&wake, period);
    }
}

/* ── Countdown (visible on serial + LED) ──────────────────────────────────── */

static void countdown(int seconds) {
    for (int i = seconds; i > 0; i--) {
        printf("#CD:%d\n", i);
        // Blink LED yellow during countdown
        led->setColor(255, 180, 0);
        led->turnOn();
        vTaskDelay(pdMS_TO_TICKS(500));
        led->turnOff();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("#CD:0\n");
    led->turnOn();
}

/* ── Start / stop helpers ──────────────────────────────────────────────────── */

static void startADCs() {
    bool wF = (mode == Mode::All || mode == Mode::Raw);
    bool wS = (mode == Mode::All || mode == Mode::Env);

    ADS1015::ChannelConfig cfg[4];
    uint8_t n = 0;
    if (wF) {
        cfg[n++] = {kEMG0, 1, ADS1015::ConfigPGA::One};
        cfg[n++] = {kEMG1, 1, ADS1015::ConfigPGA::One};
    }
    if (wS) {
        // Use divider only when fast channels are also present;
        // otherwise run the envelope channels at full speed.
        uint8_t div = wF ? kSLOW_DIV : 1;
        cfg[n++] = {kENV0, div, ADS1015::ConfigPGA::One};
        cfg[n++] = {kENV1, div, ADS1015::ConfigPGA::One};
    }
    for (int i = 0; i < 4; i++)
        adc[i]->startMixedContinuous(cfg, n, kADC_RATE);
}

static void stopADCs() {
    for (int i = 0; i < 4; i++)
        adc[i]->stopContinuous();
}

/* ── app_main ──────────────────────────────────────────────────────────────── */

extern "C" void app_main() {

    /* ---- UART driver (needed for uart_read_bytes; printf keeps working) --- */
    uart_driver_install(UART_NUM_0, 256, 0, 0, nullptr, 0);

    /* ---- RGB LED → blue while booting ------------------------------------ */
    led = new RGB(kLED);
    led->init();
    led->setColor(0, 0, 255);
    led->turnOn();

    /* ---- I2C ------------------------------------------------------------- */
    i2c0.init();
    i2c1.init();

    /* ---- ADCs ------------------------------------------------------------ */
    adc[0] = new ADS1015(i2c0, ADS1015::ADS111X_ADDR_GND);
    adc[1] = new ADS1015(i2c0, ADS1015::ADS111X_ADDR_VCC);
    adc[2] = new ADS1015(i2c1, ADS1015::ADS111X_ADDR_GND);
    adc[3] = new ADS1015(i2c1, ADS1015::ADS111X_ADDR_VCC);

    bool adcOK = true;
    for (int i = 0; i < 4; i++)
        if (!adc[i]->checkForDevice()) {
            printf("ADC%d not found\n", i + 1);
            adcOK = false;
        }

    // Configure ALERT/RDY pins + register callbacks
    for (int i = 0; i < 4; i++) {
        adc[i]->configureAlertPin(kRDY[i]);
        adc[i]->onConversion(onSample, (void*)(uintptr_t)i);
    }

    /* ---- SD card --------------------------------------------------------- */
    spiSD = new SPI(SPI::SpiMode::kMaster, SPI2_HOST, kSD_MOSI, kSD_MISO, kSD_SCK);
    if (spiSD->init() == ESP_OK) {
        sdCard = new SD(*spiSD, kSD_CS);
        if (sdCard->init() == ESP_OK && sdCard->mountCard() == FR_OK)
            sdOK = true;
    }
    if (!sdOK)
        printf("SD card init failed\n");

    /* ---- IMU (ICM-42605 over SPI3) --------------------------------------- */
    spiIMU = new SPI(SPI::SpiMode::kMaster, SPI3_HOST, kIMU_MOSI, kIMU_MISO, kIMU_SCK);
    if (spiIMU->init() == ESP_OK) {
        imu = new ICM42605(*spiIMU, kIMU_CS, 8000000);
        if (imu->init(ICM42605::AccelScale::kAFS_4G,
                       ICM42605::GyroScale::kGFS_500DPS,
                       ICM42605::AccelODR::kAODR_200Hz,
                       ICM42605::GyroODR::kGODR_200Hz) == ESP_OK) {
            imuOK = true;
            printf("IMU OK (ICM-42605 SPI)\n");
        } else {
            printf("IMU init failed\n");
        }
    } else {
        printf("IMU SPI bus init failed\n");
    }

    /* ---- Reed switch ----------------------------------------------------- */
    reedSw = new Switch(kREED, Switch::SwitchMode::kNormallyOpen);
    reedSw->init();

    /* ---- Battery manager (Carguita 5V) ----------------------------------- */
    battery = new BatteryManager(kBAT_ADC, kCHG, kPGOOD, k5V_EN);
    if (battery->init() != ESP_OK)
        printf("Battery manager init failed\n");
    else
        battery->measure();   // initial reading

    /* ---- Sample queues --------------------------------------------------- */
    sdQ  = xQueueCreate(kQLEN, sizeof(Sample));
    imuQ = xQueueCreate(kIMU_QLEN, sizeof(ImuSample));

    /* ---- Status LED ------------------------------------------------------ */
    if (!adcOK || !sdOK) {
        led->setColor(255, 0, 0);   // red = init error
        printf("#INIT:ADC=%s,SD=%s,IMU=%s\n", adcOK ? "OK" : "FAIL",
               sdOK ? "OK" : "FAIL", imuOK ? "OK" : "FAIL");
    } else {
        led->setColor(0, 255, 0);   // green = all good
    }

    /* ==== Wait for command ================================================ */
    printf("#READY\n");

    bool reedPrev = reedSw->isPressed();
    uint8_t rx;

    while (mode == Mode::Idle) {
        // Check UART
        if (uart_read_bytes(UART_NUM_0, &rx, 1, pdMS_TO_TICKS(50)) > 0) {
            if (rx == '1') mode = Mode::All;
            else if (rx == '2') mode = Mode::Raw;
            else if (rx == '3') mode = Mode::Env;
            else if (rx == '?') {
                uint16_t mv = 0;
                uint8_t pct = 0;
                if (battery) { battery->measure(); mv = battery->getVoltage(); pct = battery->getPercentage(); }
                printf("#STATUS:0,%d,%u,%u\n", 0, mv, pct);
            }
        }
        // Check reed switch (rising edge)
        bool reedNow = reedSw->isPressed();
        if (reedNow && !reedPrev)
            mode = Mode::All;
        reedPrev = reedNow;
    }

    /* ==== Start recording ================================================= */
    printf("#MODE:%d\n", (int)mode);
    countdown(3);
    battery->enable5V();

    recStart  = esp_timer_get_time();
    recording = true;
    printf("#REC\n");

    if (sdOK)
        xTaskCreatePinnedToCore(sdWriteTask, "sd",   8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(uartTask,     "uart", 4096, nullptr, 3, nullptr, 0);
    if (imuOK)
        xTaskCreatePinnedToCore(imuTask,  "imu",  4096, nullptr, 4, nullptr, 1);

    startADCs();

    /* ==== Main loop: monitor reed + UART ================================== */
    constexpr uint32_t kDebounceMs = 300;
    uint32_t lastToggle = 0;

    while (true) {
        uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);

        // ---- Reed switch toggle (with debounce) ----
        bool reedNow = reedSw->isPressed();
        if (reedNow && !reedPrev && (nowMs - lastToggle > kDebounceMs)) {
            lastToggle = nowMs;
            if (recording) {
                recording = false;
                stopADCs();
                battery->disable5V();
                led->setColor(0, 0, 255);
                printf("#PAUSE\n");
            } else {
                countdown(3);
                battery->enable5V();
                recStart  = esp_timer_get_time();
                recording = true;
                startADCs();
                led->setColor(0, 255, 0);
                printf("#REC\n");
            }
        }
        reedPrev = reedNow;

        // ---- UART commands ----
        if (uart_read_bytes(UART_NUM_0, &rx, 1, pdMS_TO_TICKS(50)) > 0) {
            if (rx == '0') {
                // Stop
                recording = false;
                stopADCs();
                battery->disable5V();
                led->setColor(0, 0, 255);
                printf("#STOP\n");
            } else if (rx >= '1' && rx <= '3') {
                Mode newM = (Mode)(rx - '0');
                if (newM != mode || !recording) {
                    if (recording) stopADCs();
                    printf("#MODE:%d\n", (int)newM);
                    countdown(3);
                    battery->enable5V();
                    mode      = newM;
                    recStart  = esp_timer_get_time();
                    recording = true;
                    startADCs();
                    led->setColor(0, 255, 0);
                    printf("#REC\n");
                }
            } else if (rx == '?') {
                uint16_t mv = 0;
                uint8_t pct = 0;
                if (battery) { battery->measure(); mv = battery->getVoltage(); pct = battery->getPercentage(); }
                printf("#STATUS:%d,%d,%u,%u\n", (int)mode, recording ? 1 : 0, mv, pct);
            }
        }
    }
}