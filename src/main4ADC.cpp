/*******************************************************************************
 * main4ADC.cpp — EMG8 Bracelet Firmware
 *
 * 4× ADS1015 mixed-rate continuous sampling with ALERT/RDY interrupts:
 *   ch 0, 1 = fast  (raw EMG   ≈ 1150 Hz per channel in All mode)
 *   ch 2, 3 = slow  (envelope  ≈   57 Hz per channel in All mode)
 *
 * Binary SD logging  ·  UART CSV output  ·  Reed switch + serial commands
 *
 * SD binary format (per file):
 *   [16-byte header] "EMG8" | ver(1) | nADC(4) | nCh(4) | div | epoch_s(u32)
 *   [N × 8-byte records] timestamp_us(u32) | adc(u8) | ch(u8) | value(i16)
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
#include "actuators_sense.hpp"
#include "switch_sense.hpp"
#include "sd_storage_sense.hpp"
#include "spi_gp_sense.hpp"

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

/* ── Channel layout (identical on every ADC) ───────────────────────────────── */

static constexpr uint8_t kEMG0     = 0;   // fast — raw EMG
static constexpr uint8_t kEMG1     = 1;   // fast — raw EMG
static constexpr uint8_t kENV0     = 2;   // slow — envelope
static constexpr uint8_t kENV1     = 3;   // slow — envelope
static constexpr uint8_t kSLOW_DIV = 20;  // ≈ 57 Hz envelope at 2400 SPS w/ 2 fast ch
static constexpr auto    kADC_RATE = ADS1015::ConfigRate::Rate_2400Hz;

/* ── Types ─────────────────────────────────────────────────────────────────── */

struct __attribute__((packed)) Sample {
    uint32_t ts;    // µs since recording start
    uint8_t  adc;   // 0–3
    uint8_t  ch;    // 0–3
    int16_t  val;   // signed 12-bit
};
static_assert(sizeof(Sample) == 8, "Sample must be 8 bytes");

enum class Mode : uint8_t { Idle = 0, All = 1, Raw = 2, Env = 3 };

/* ── Globals ───────────────────────────────────────────────────────────────── */

static I2C i2c0(I2C_NUM_0, kSDA0, kSCL0, 1000000, false);
static I2C i2c1(I2C_NUM_1, kSDA1, kSCL1, 1000000, false);

static ADS1015* adc[4];
static SPI*     spiSD  = nullptr;
static SD*      sdCard = nullptr;
static RGB*     led    = nullptr;
static Switch*  reedSw = nullptr;

static volatile Mode mode      = Mode::Idle;
static volatile bool recording = false;
static int64_t       recStart  = 0;            // µs epoch for timestamps
static bool          sdOK      = false;        // SD card available

static QueueHandle_t sdQ = nullptr;
static constexpr int kQLEN = 8000;             // ≈ 600 ms buffer at 13.2 kSa/s

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
    Sample buf[BATCH];
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

            // 16-byte header
            uint8_t hdr[16] = {};
            memcpy(hdr, "EMG8", 4);
            hdr[4] = 1;           // version
            hdr[5] = 4;           // number of ADCs
            hdr[6] = 4;           // channels per ADC
            hdr[7] = kSLOW_DIV;   // slow divider
            uint32_t e32 = (uint32_t)fEpoch;
            memcpy(hdr + 8, &e32, 4);
            sdCard->fileWrite(std::string(reinterpret_cast<char*>(hdr), 16));
            fOpen = true;
        } else if (nowSec - fEpoch >= 60) {
            sdCard->closeFile();
            fOpen = false;
            continue;   // will reopen on next iteration
        }

        /* ---- drain queue → batch write ---- */
        int n = 0;
        while (n < BATCH && xQueueReceive(sdQ, &buf[n], 0) == pdTRUE) n++;
        if (n > 0) {
            sdCard->fileWrite(
                std::string(reinterpret_cast<char*>(buf), n * sizeof(Sample)));
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));  // yield when idle
        }
    }
}

/* ── UART CSV task — pinned to core 0, ~50 Hz ─────────────────────────────── */

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

        // Print header line once per recording start
        if (!hdrDone) {
            bool first = true;
            for (int a = 0; a < 4; a++)
                for (int c = 0; c < 4; c++) {
                    bool fast = (c <= kEMG1);
                    if (mode == Mode::Raw && !fast) continue;
                    if (mode == Mode::Env &&  fast) continue;
                    printf("%sadc%d_%d", first ? "" : ",", a + 1, c);
                    first = false;
                }
            printf("\n");
            lastHdrMode = mode;
            hdrDone = true;
        }

        // One CSV line of latest readings
        char line[256];
        int  p = 0;
        bool first = true;
        for (int a = 0; a < 4; a++)
            for (int c = 0; c < 4; c++) {
                bool fast = (c <= kEMG1);
                if (mode == Mode::Raw && !fast) continue;
                if (mode == Mode::Env &&  fast) continue;
                if (!first) line[p++] = ',';
                p += snprintf(line + p, sizeof(line) - p, "%d",
                              adc[a]->getLatestReading(c));
                first = false;
            }
        line[p++] = '\n';
        line[p] = '\0';
        printf("%s", line);

        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz
    }
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
    led = new RGB();
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

    /* ---- Reed switch ----------------------------------------------------- */
    reedSw = new Switch(kREED, Switch::SwitchMode::kNormallyOpen);
    reedSw->init();

    /* ---- Sample queue ---------------------------------------------------- */
    sdQ = xQueueCreate(kQLEN, sizeof(Sample));

    /* ---- Status LED ------------------------------------------------------ */
    if (!adcOK || !sdOK) {
        led->setColor(255, 0, 0);   // red = init error
        printf("INIT  ADC=%s  SD=%s\n", adcOK ? "OK" : "FAIL", sdOK ? "OK" : "FAIL");
    } else {
        led->setColor(0, 255, 0);   // green = all good
    }

    /* ==== Wait for command ================================================ */
    printf("\nEMG8 ready.  Commands: 1=All  2=Raw  3=Env  (or press reed)\n");

    bool reedPrev = reedSw->isPressed();
    uint8_t rx;

    while (mode == Mode::Idle) {
        // Check UART
        if (uart_read_bytes(UART_NUM_0, &rx, 1, pdMS_TO_TICKS(50)) > 0) {
            if (rx == '1') mode = Mode::All;
            else if (rx == '2') mode = Mode::Raw;
            else if (rx == '3') mode = Mode::Env;
        }
        // Check reed switch (rising edge)
        bool reedNow = reedSw->isPressed();
        if (reedNow && !reedPrev)
            mode = Mode::All;
        reedPrev = reedNow;
    }

    /* ==== Start recording ================================================= */
    printf("Mode %d — starting\n", (int)mode);
    led->setColor(0, 255, 0);

    recStart  = esp_timer_get_time();
    recording = true;

    if (sdOK)
        xTaskCreatePinnedToCore(sdWriteTask, "sd",   8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(uartTask,     "uart", 4096, nullptr, 3, nullptr, 0);

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
                led->setColor(0, 0, 255);
                printf("Paused\n");
            } else {
                recStart  = esp_timer_get_time();
                recording = true;
                startADCs();
                led->setColor(0, 255, 0);
                printf("Resumed\n");
            }
        }
        reedPrev = reedNow;

        // ---- UART commands ----
        if (uart_read_bytes(UART_NUM_0, &rx, 1, pdMS_TO_TICKS(50)) > 0) {
            if (rx == '0') {
                // Stop
                recording = false;
                stopADCs();
                led->setColor(0, 0, 255);
                printf("Stopped\n");
            } else if (rx >= '1' && rx <= '3') {
                Mode newM = (Mode)(rx - '0');
                if (newM != mode || !recording) {
                    if (recording) stopADCs();
                    mode      = newM;
                    recStart  = esp_timer_get_time();
                    recording = true;
                    startADCs();
                    led->setColor(0, 255, 0);
                    printf("Mode %d\n", (int)mode);
                }
            }
        }
    }
}