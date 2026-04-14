/*******************************************************************************
 * main4ADC.cpp — EMG8 Bracelet Firmware  (v4 — multi-file SD + label protocol)
 *
 * 4× ADS1015 mixed-rate continuous sampling with ALERT/RDY interrupts:
 *   ch 0, 2 = fast  (raw EMG   ≈ 1150 Hz per channel in All mode)
 *   ch 1, 3 = slow  (envelope  ≈   57 Hz per channel in All mode)
 *
 * ICM-42605 6-axis IMU at 200 Hz via SPI
 *
 * Multi-file SD logging  ·  UART CSV @460800  ·  Reed switch + serial commands
 *
 * SD layout per session directory  s_<MAC>_<epoch>/
 *   M.bin — master  (32-byte header + 12-byte label events)
 *   R.bin — raw EMG  (8-byte Sample records, ch 0-1)
 *   E.bin — envelope (8-byte Sample records, ch 2-3)
 *   I.bin — IMU      (20-byte ImuSample records)
 *
 * UART baud 460800.  Protocol:
 *   PC→ESP: '0' stop · '1-3' mode · '?' status · 'L<id>,<rep>\n' label
 *           'V0'/'V1' 5V · 'F' list files · 'G<path>\n' transfer file
 *   ESP→PC: #READY · #MODE:N · #CD:N · #REC · #STOP · #LABEL:id,rep
 *           #STATUS:mode,rec,mV,% · H,… · D,ts,… · #5V:0/1
 *
 * Sense-AI
 ******************************************************************************/
#include <cstring>
#include <string>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_mac.h"
#include "ff.h"

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
static constexpr uint8_t kEMG1     = 2;   // fast — raw EMG
static constexpr uint8_t kENV0     = 1;   // slow — envelope
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

/** @brief Label event written to master file when PC sends L command. */
struct __attribute__((packed)) LabelEvent {
    uint32_t ts;           // µs since recStart
    uint16_t grasp_id;     // Ninapro movement number (0 = rest)
    uint16_t repetition;   // current repetition
    uint32_t _reserved;    // pad to 12 bytes
};
static_assert(sizeof(LabelEvent) == 12, "LabelEvent must be 12 bytes");

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
static std::atomic<int64_t> recStart{0};     // µs epoch for timestamps
static bool          sdOK      = false;        // SD card available

// Separate queues for each stream → separate files
static QueueHandle_t rawQ   = nullptr;         // raw EMG (ch 0,1)
static QueueHandle_t envQ   = nullptr;         // envelope (ch 2,3)
static QueueHandle_t imuQ   = nullptr;
static QueueHandle_t labelQ = nullptr;         // LabelEvent from PC
static constexpr int kRAW_QLEN  = 8000;        // ≈ 600 ms @ ~13 kSa/s
static constexpr int kENV_QLEN  = 1000;        // ≈ 2 s   @ ~460 Sa/s
static constexpr int kIMU_QLEN  = 400;         // ≈ 2 s   @ 200 Hz
static constexpr int kLABEL_QLEN = 32;

// Label state (set by PC via 'L' command)
static volatile uint16_t curGrasp = 0;         // current Ninapro grasp ID
static volatile uint16_t curRep   = 0;         // current repetition

// Device MAC as hex string  "AABBCCDDEEFF\0"
static char macStr[13] = {};

// Session directory path (set once at SD init)
static std::string sessionDir;

/* ── ADC conversion callback (called from each ADC's FreeRTOS task) ──────── */

static void onSample(uint8_t ch, int16_t val, void* arg) {
    uint8_t id = (uint8_t)(uintptr_t)arg;
    bool fast  = (ch == kEMG0 || ch == kEMG1);

    // Drop channels the current mode doesn't need
    if (mode == Mode::Raw && !fast) return;
    if (mode == Mode::Env &&  fast) return;

    Sample s;
    s.ts  = (uint32_t)(esp_timer_get_time() - recStart);
    s.adc = id;
    s.ch  = ch;
    s.val = val;

    if (fast)
        xQueueSend(rawQ, &s, 0);
    else
        xQueueSend(envQ, &s, 0);
}

/* ── SD writer task — multi-file via raw FatFs, pinned to core 1, prio 5 ── */
/*
 * Files opened once per recording session (no open/close cycling):
 *   M.bin — master header + label events
 *   R.bin — raw EMG  (8-byte Sample, ch 0-1)
 *   E.bin — envelope (8-byte Sample, ch 2-3)
 *   I.bin — IMU      (20-byte ImuSample)
 *
 * Uses raw f_open/f_write/f_sync to hold 4 FIL handles simultaneously
 * (the SD wrapper class only supports one open file at a time).
 */

static FIL filMaster, filRaw, filEnv, filImu;
static bool filesOpen = false;

static void sdOpenFiles(const std::string& base) {
    std::string mPath = base + "/M.bin";
    std::string rPath = base + "/R.bin";
    std::string ePath = base + "/E.bin";
    std::string iPath = base + "/I.bin";

    f_open(&filMaster, mPath.c_str(), FA_CREATE_ALWAYS | FA_WRITE);
    f_open(&filRaw,    rPath.c_str(), FA_CREATE_ALWAYS | FA_WRITE);
    f_open(&filEnv,    ePath.c_str(), FA_CREATE_ALWAYS | FA_WRITE);
    f_open(&filImu,    iPath.c_str(), FA_CREATE_ALWAYS | FA_WRITE);

    // Write master header (32 bytes, v4)
    // [0-3] "EMG8"  [4] ver=4  [5] nADC  [6] nCh  [7] div
    // [8-11] epoch_s(u32)  [12-13] bat_mV  [14] bat_%  [15] bat_state
    // [16-17] imuODR(u16)  [18-23] MAC(6)  [24] mode  [25-31] reserved
    uint8_t hdr[32] = {};
    memcpy(hdr, "EMG8", 4);
    hdr[4] = 4;                  // version
    hdr[5] = 4;                  // number of ADCs
    hdr[6] = 4;                  // channels per ADC
    hdr[7] = kSLOW_DIV;
    uint32_t e32 = (uint32_t)(esp_timer_get_time() / 1000000);
    memcpy(hdr + 8, &e32, 4);
    if (battery) {
        battery->measure();
        uint16_t mv = battery->getVoltage();
        memcpy(hdr + 12, &mv, 2);
        hdr[14] = battery->getPercentage();
        hdr[15] = (uint8_t)battery->getState();
    }
    uint16_t odr16 = kIMU_ODR_HZ;
    memcpy(hdr + 16, &odr16, 2);
    // MAC bytes
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    memcpy(hdr + 18, mac, 6);
    hdr[24] = (uint8_t)mode;

    UINT bw;
    f_write(&filMaster, hdr, 32, &bw);
    f_sync(&filMaster);

    filesOpen = true;
}

static void sdCloseFiles() {
    if (!filesOpen) return;
    f_sync(&filRaw);  f_close(&filRaw);
    f_sync(&filEnv);  f_close(&filEnv);
    f_sync(&filImu);  f_close(&filImu);
    f_sync(&filMaster); f_close(&filMaster);
    filesOpen = false;
}

static void sdWriteTask(void*) {
    constexpr int RAW_BATCH = 500;    // 500 × 8  = 4 KB
    constexpr int ENV_BATCH = 100;    // 100 × 8  = 800 B
    constexpr int IMU_BATCH = 50;     //  50 × 20 = 1 KB
    constexpr int LBL_BATCH = 8;

    Sample    rawBuf[RAW_BATCH];
    Sample    envBuf[ENV_BATCH];
    ImuSample imuBuf[IMU_BATCH];
    LabelEvent lblBuf[LBL_BATCH];
    UINT bw;

    uint32_t syncTick = 0;

    while (true) {
        /* ---- idle when not recording ---- */
        if (!recording) {
            if (filesOpen) sdCloseFiles();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ---- open files on first iteration of a new recording ---- */
        if (!filesOpen) {
            sdOpenFiles(sessionDir);
        }

        /* ---- drain raw EMG queue → R.bin ---- */
        int nR = 0;
        while (nR < RAW_BATCH && xQueueReceive(rawQ, &rawBuf[nR], 0) == pdTRUE) nR++;
        if (nR > 0)
            f_write(&filRaw, rawBuf, nR * sizeof(Sample), &bw);

        /* ---- drain envelope queue → E.bin ---- */
        int nE = 0;
        while (nE < ENV_BATCH && xQueueReceive(envQ, &envBuf[nE], 0) == pdTRUE) nE++;
        if (nE > 0)
            f_write(&filEnv, envBuf, nE * sizeof(Sample), &bw);

        /* ---- drain IMU queue → I.bin ---- */
        int nI = 0;
        while (nI < IMU_BATCH && xQueueReceive(imuQ, &imuBuf[nI], 0) == pdTRUE) nI++;
        if (nI > 0)
            f_write(&filImu, imuBuf, nI * sizeof(ImuSample), &bw);

        /* ---- drain label queue → M.bin ---- */
        int nL = 0;
        while (nL < LBL_BATCH && xQueueReceive(labelQ, &lblBuf[nL], 0) == pdTRUE) nL++;
        if (nL > 0)
            f_write(&filMaster, lblBuf, nL * sizeof(LabelEvent), &bw);

        /* ---- periodic sync (every ~500 ms) ---- */
        if (nR || nE || nI || nL) {
            if (++syncTick >= 25) {   // 25 × 20ms ≈ 500ms
                f_sync(&filRaw);
                f_sync(&filEnv);
                f_sync(&filImu);
                f_sync(&filMaster);
                syncTick = 0;
            }
        }

        if (nR == 0 && nE == 0 && nI == 0 && nL == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));  // yield when idle
        }
    }
}

/* ── UART output task — pinned to core 0, ~50 Hz ──────────────────────────── */
/*  Protocol: line-based text, prefixed by a tag character.
 *
 *  Outgoing (ESP → PC):
 *    '#' lines = metadata / status
 *    'H' line  = CSV header   H,ts_us,adc1_0,...,ax,ay,az,gx,gy,gz,label,rep
 *    'D' line  = CSV data     D,12345,1234,-456,...,0.12,-0.03,0.98,...,5,2
 *
 *  Incoming (PC → ESP):
 *    '0'        stop
 *    '1'-'3'    mode
 *    '?'        query status
 *    'L<id>,<rep>\n'  set label
 *    'V0'/'V1'  5V off/on
 *    'F'        list SD files
 *    'G<path>\n' transfer SD file
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
            printf("H,ts_us");
            for (int a = 0; a < 4; a++)
                for (int c = 0; c < 4; c++) {
                    bool fast = (c == kEMG0 || c == kEMG1);
                    if (mode == Mode::Raw && !fast) continue;
                    if (mode == Mode::Env &&  fast) continue;
                    printf(",adc%d_%d", a + 1, c);
                }
            if (imuOK)
                printf(",ax,ay,az,gx,gy,gz");
            printf(",label,rep\n");
            lastHdrMode = mode;
            hdrDone = true;
        }

        // One CSV line of latest readings
        char line[512];
        int  p = 0;

        // Timestamp (µs since recStart)
        uint32_t ts = (uint32_t)(esp_timer_get_time() - recStart);
        p += snprintf(line + p, sizeof(line) - p, "D,%u", (unsigned)ts);

        for (int a = 0; a < 4; a++)
            for (int c = 0; c < 4; c++) {
                bool fast = (c == kEMG0 || c == kEMG1);
                if (mode == Mode::Raw && !fast) continue;
                if (mode == Mode::Env &&  fast) continue;
                p += snprintf(line + p, sizeof(line) - p, ",%d",
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
        // Append current label
        p += snprintf(line + p, sizeof(line) - p, ",%u,%u",
                      (unsigned)curGrasp, (unsigned)curRep);
        line[p++] = '\n';
        line[p] = '\0';
        printf("%s", line);

        vTaskDelay(pdMS_TO_TICKS(20));  // ~50 Hz
    }
}

/* ── Single ADC polling task — handles all 4 ADCs from one task ────────────── */
/*
 * Replaces the 4 internal per-ADC tasks. Each ADC's ISR sets a binary semaphore
 * on DRDY; this task polls all four with non-blocking takes. When no conversion
 * is ready, it yields for 1 tick (1 ms at 1000 Hz tick rate) to avoid starving
 * IDLE1. Worst-case added latency: 1 ms, negligible at 2400 SPS.
 */

static void adcTask(void*) {
    while (true) {
        if (!recording) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        bool anyServiced = false;
        for (int i = 0; i < 4; i++) {
            if (adc[i]->serviceConversion()) {
                anyServiced = true;
            }
        }

        if (!anyServiced) {
            vTaskDelay(1);  // yield 1 tick when no ADC has data
        }
    }
}

/* ── IMU polling task — pinned to core 1, 200 Hz ──────────────────────────── */

static void imuTask(void*) {
    constexpr TickType_t period =
        pdMS_TO_TICKS(1000 / kIMU_ODR_HZ) > 0 ? pdMS_TO_TICKS(1000 / kIMU_ODR_HZ) : 1;
    TickType_t wake = xTaskGetTickCount();

    while (true) {
        if (!recording || !imuOK) {
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

/* ── Countdown (visible on serial + LED, interruptible via '0') ────────────── */

static bool countdown(int seconds) {
    for (int i = seconds; i > 0; i--) {
        printf("#CD:%d\n", i);
        led->setColor(255, 180, 0);
        led->turnOn();
        // 10 × 100 ms = 1 second; check UART each tick
        for (int t = 0; t < 10; t++) {
            uint8_t rx;
            if (uart_read_bytes(UART_NUM_0, &rx, 1, pdMS_TO_TICKS(100)) > 0) {
                if (rx == '0') {
                    led->turnOff();
                    printf("#CD:ABORT\n");
                    return false;   // aborted
                }
            }
            if (t == 5) led->turnOff();   // blink: 500ms on, 500ms off
        }
    }
    printf("#CD:0\n");
    led->turnOn();
    return true;   // completed normally
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
        adc[i]->startMixedContinuousExternal(cfg, n, kADC_RATE);
}

static void stopADCs() {
    for (int i = 0; i < 4; i++)
        adc[i]->stopContinuous();
}

/* ── UART line buffer for multi-byte commands ──────────────────────────────── */

static char uartLineBuf[128];
static int  uartLinePos = 0;

/** Process a complete line command (after '\n'). */
static void processUartLine(const char* line, int len) {
    if (len < 1) return;

    if (line[0] == 'L') {
        // Label: L<grasp_id>,<rep>
        int gid = 0, rep = 0;
        if (sscanf(line + 1, "%d,%d", &gid, &rep) >= 1) {
            curGrasp = (uint16_t)gid;
            curRep   = (uint16_t)rep;
            printf("#LABEL:%d,%d\n", gid, rep);
            // Enqueue label event for SD master file
            if (recording && labelQ) {
                LabelEvent le = {};
                le.ts = (uint32_t)(esp_timer_get_time() - recStart);
                le.grasp_id = (uint16_t)gid;
                le.repetition = (uint16_t)rep;
                xQueueSend(labelQ, &le, 0);
            }
        }
    } else if (line[0] == 'G') {
        // File transfer: G<relative_path>
        if (!sdOK) { printf("#ERR:NO_SD\n"); return; }
        std::string fpath(line + 1, len - 1);
        // Trim trailing whitespace
        while (!fpath.empty() && (fpath.back() == '\r' || fpath.back() == ' '))
            fpath.pop_back();

        FIL tf;
        if (f_open(&tf, fpath.c_str(), FA_READ) != FR_OK) {
            printf("#ERR:OPEN_FAIL\n");
            return;
        }
        uint32_t sz = f_size(&tf);
        printf("#FDATA:%s,%u\n", fpath.c_str(), (unsigned)sz);

        uint8_t xbuf[512];
        UINT br;
        while (f_read(&tf, xbuf, sizeof(xbuf), &br) == FR_OK && br > 0) {
            // Send raw binary (the Python side reads exactly sz bytes)
            uart_write_bytes(UART_NUM_0, xbuf, br);
        }
        f_close(&tf);
        printf("\n#FDONE\n");
    }
}

/** Feed one byte to the UART line accumulator. Returns single-char cmds. */
static int feedUartByte(uint8_t b) {
    // Multi-byte commands start with 'L', 'G' and end with '\n'
    if (uartLinePos > 0) {
        // We're accumulating a line
        if (b == '\n' || b == '\r') {
            uartLineBuf[uartLinePos] = '\0';
            processUartLine(uartLineBuf, uartLinePos);
            uartLinePos = 0;
            return 0;   // consumed
        }
        if (uartLinePos < (int)sizeof(uartLineBuf) - 1)
            uartLineBuf[uartLinePos++] = (char)b;
        return 0;   // consumed
    }

    // First byte of a potential command
    if (b == 'L' || b == 'G') {
        uartLineBuf[0] = (char)b;
        uartLinePos = 1;
        return 0;
    }

    // Single-byte commands
    return b;
}

/* ── SD directory listing helper ───────────────────────────────────────────── */

static void listSDDir(const char* path) {
    FF_DIR dir;
    FILINFO fno;
    if (f_opendir(&dir, path) != FR_OK) {
        printf("#ERR:DIR\n");
        return;
    }
    printf("#FLIST:%s\n", path);
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (fno.fname[0] == '.') continue;
        bool isDir = (fno.fattrib & AM_DIR);
        printf("#F:%s%s,%u\n", fno.fname, isDir ? "/" : "",
               isDir ? 0u : (unsigned)fno.fsize);
        if (isDir) {
            // Recurse one level for session directories
            char sub[300];
            snprintf(sub, sizeof(sub), "%.*s/%.*s",
                     (int)(sizeof(sub)/2 - 2), path,
                     (int)(sizeof(sub)/2 - 2), fno.fname);
            FF_DIR sub_dir;
            FILINFO sub_fno;
            if (f_opendir(&sub_dir, sub) == FR_OK) {
                while (f_readdir(&sub_dir, &sub_fno) == FR_OK && sub_fno.fname[0] != '\0') {
                    if (sub_fno.fname[0] == '.') continue;
                    printf("#F:%s/%s,%u\n", fno.fname, sub_fno.fname,
                           (unsigned)sub_fno.fsize);
                }
                f_closedir(&sub_dir);
            }
        }
    }
    f_closedir(&dir);
    printf("#FEND\n");
}

/* ── app_main ──────────────────────────────────────────────────────────────── */

extern "C" void app_main() {

    /* ---- UART driver @ 460800 baud --------------------------------------- */
    const uart_config_t uart_cfg = {
        .baud_rate  = 460800,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags      = {},
    };
    uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
    uart_param_config(UART_NUM_0, &uart_cfg);

    /* ---- Device MAC → hex string ----------------------------------------- */
    {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

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
    esp_err_t sdSpiErr = spiSD->init();
    if (sdSpiErr != ESP_OK) {
        printf("SD SPI bus init failed: %s\n", esp_err_to_name(sdSpiErr));
    } else {
        printf("SD SPI bus OK (MOSI=%d MISO=%d SCK=%d CS=%d)\n",
               kSD_MOSI, kSD_MISO, kSD_SCK, kSD_CS);
        sdCard = new SD(*spiSD, kSD_CS);
        esp_err_t sdErr = sdCard->init();
        if (sdErr != ESP_OK) {
            printf("SD card init failed: %s (0x%x)\n", esp_err_to_name(sdErr), sdErr);
        } else {
            FRESULT fr = sdCard->mountCard();
            if (fr != FR_OK) {
                printf("SD mount failed: FRESULT=%d\n", fr);
            } else {
                sdOK = true;
                printf("SD card mounted OK\n");
            }
        }
    }
    if (!sdOK)
        printf("SD card NOT available\n");

    // Create session directory using MAC: s_<MAC>_<epoch>
    if (sdOK) {
        char sdir[48];
        snprintf(sdir, sizeof(sdir), "s_%s_%lld", macStr,
                 (long long)(esp_timer_get_time() / 1000000));
        sdCard->createDir(std::string(sdir));
        sdCard->goToDir(std::string(sdir));
        sessionDir = sdCard->getCurrentDir();
    }

    /* ---- IMU (ICM-42605 over SPI3) --------------------------------------- */
    esp_log_level_set("ICM42605", ESP_LOG_DEBUG);
    spiIMU = new SPI(SPI::SpiMode::kMaster, SPI3_HOST, kIMU_MOSI, kIMU_MISO, kIMU_SCK);
    esp_err_t imuSpiErr = spiIMU->init();
    if (imuSpiErr != ESP_OK) {
        printf("IMU SPI bus init failed: %s\n", esp_err_to_name(imuSpiErr));
    } else {
        printf("IMU SPI bus OK (MOSI=%d MISO=%d SCK=%d CS=%d)\n",
               kIMU_MOSI, kIMU_MISO, kIMU_SCK, kIMU_CS);
        imu = new ICM42605(*spiIMU, kIMU_CS, 1000000);  // 1 MHz
        vTaskDelay(pdMS_TO_TICKS(50));  // Let SPI bus + IMU settle before init
        esp_err_t imuErr = imu->init(ICM42605::AccelScale::kAFS_4G,
                       ICM42605::GyroScale::kGFS_500DPS,
                       ICM42605::AccelODR::kAODR_200Hz,
                       ICM42605::GyroODR::kGODR_200Hz);
        if (imuErr == ESP_OK) {
            imuOK = true;
            printf("IMU OK (ICM-42605 SPI)\n");
        } else {
            printf("IMU init failed: %s (0x%x)\n", esp_err_to_name(imuErr), imuErr);
        }
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
    rawQ   = xQueueCreate(kRAW_QLEN,   sizeof(Sample));
    envQ   = xQueueCreate(kENV_QLEN,   sizeof(Sample));
    imuQ   = xQueueCreate(kIMU_QLEN,   sizeof(ImuSample));
    labelQ = xQueueCreate(kLABEL_QLEN, sizeof(LabelEvent));

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
    printf("#MAC:%s\n", macStr);

    bool reedPrev = reedSw->isPressed();
    uint8_t rxByte;

    while (mode == Mode::Idle) {
        // Check UART
        if (uart_read_bytes(UART_NUM_0, &rxByte, 1, pdMS_TO_TICKS(50)) > 0) {
            int cmd = feedUartByte(rxByte);
            if (cmd == '1') mode = Mode::All;
            else if (cmd == '2') mode = Mode::Raw;
            else if (cmd == '3') mode = Mode::Env;
            else if (cmd == '?') {
                uint16_t mv = 0; uint8_t pct = 0;
                if (battery) { battery->measure(); mv = battery->getVoltage(); pct = battery->getPercentage(); }
                printf("#STATUS:0,%d,%u,%u\n", 0, mv, pct);
            }
            else if (cmd == 'V') {
                // Read next byte for V0/V1
                uint8_t vb;
                if (uart_read_bytes(UART_NUM_0, &vb, 1, pdMS_TO_TICKS(100)) > 0) {
                    if (vb == '1' && battery) { battery->enable5V(); printf("#5V:1\n"); }
                    else if (vb == '0' && battery) { battery->disable5V(); printf("#5V:0\n"); }
                }
            }
            else if (cmd == 'F') {
                if (sdOK) {
                    std::string root = sdCard->getCurrentDir();
                    // Go up to root for listing
                    listSDDir(sessionDir.substr(0, sessionDir.find_last_of('/')).c_str());
                }
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
    if (!countdown(3)) {
        // Countdown aborted → go back to idle
        mode = Mode::Idle;
        printf("#STOP\n");
        // Fall through to main loop but not recording
    }

    if (mode != Mode::Idle) {
        battery->enable5V();
        recStart  = esp_timer_get_time();
        recording = true;
        printf("#REC\n");
    }

    if (sdOK)
        xTaskCreatePinnedToCore(sdWriteTask, "sd",   8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(uartTask,     "uart", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(adcTask,      "adc",  4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
    if (imuOK)
        xTaskCreatePinnedToCore(imuTask,  "imu",  4096, nullptr, 4, nullptr, 1);

    if (recording)
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
                if (countdown(3)) {
                    battery->enable5V();
                    recStart  = esp_timer_get_time();
                    recording = true;
                    startADCs();
                    led->setColor(0, 255, 0);
                    printf("#REC\n");
                }
            }
        }
        reedPrev = reedNow;

        // ---- UART commands ----
        if (uart_read_bytes(UART_NUM_0, &rxByte, 1, pdMS_TO_TICKS(50)) > 0) {
            int cmd = feedUartByte(rxByte);
            if (cmd == 0) continue;   // consumed by line accumulator

            if (cmd == '0') {
                recording = false;
                stopADCs();
                battery->disable5V();
                led->setColor(0, 0, 255);
                printf("#STOP\n");
            } else if (cmd >= '1' && cmd <= '3') {
                Mode newM = (Mode)(cmd - '0');
                if (newM != mode || !recording) {
                    if (recording) stopADCs();
                    printf("#MODE:%d\n", (int)newM);
                    if (countdown(3)) {
                        battery->enable5V();
                        mode      = newM;
                        recStart  = esp_timer_get_time();
                        recording = true;
                        startADCs();
                        led->setColor(0, 255, 0);
                        printf("#REC\n");
                    } else {
                        printf("#STOP\n");
                    }
                }
            } else if (cmd == '?') {
                uint16_t mv = 0; uint8_t pct = 0;
                if (battery) { battery->measure(); mv = battery->getVoltage(); pct = battery->getPercentage(); }
                printf("#STATUS:%d,%d,%u,%u\n", (int)mode, recording ? 1 : 0, mv, pct);
            } else if (cmd == 'V') {
                uint8_t vb;
                if (uart_read_bytes(UART_NUM_0, &vb, 1, pdMS_TO_TICKS(100)) > 0) {
                    if (vb == '1' && battery) { battery->enable5V(); printf("#5V:1\n"); }
                    else if (vb == '0' && battery) { battery->disable5V(); printf("#5V:0\n"); }
                }
            } else if (cmd == 'F') {
                if (sdOK) {
                    listSDDir(sessionDir.substr(0, sessionDir.find_last_of('/')).c_str());
                }
            }
        }
    }
}