# EMG8 Bracelet — Improvements Work-Plan

> Generated 2026-04-13 from firmware review + crash log analysis.
> Refer to git blame on `src/main.cpp` and the ADS1015 / ICM42605 drivers for context.

---

## P0 — CRASHES & WATCHDOG (fix before any data collection)

### P0-1 · `xTaskDelayUntil` assert → panic reboot

**Symptom**
```
assert failed: xTaskDelayUntil IDF\components\freertos\FreeRTOS-Kernel\tasks.c:1476
  (( xTimeIncrement > 0U ))
```

**Root cause**
`CONFIG_FREERTOS_HZ=100` (10 ms tick). The IMU task computes:
```cpp
const TickType_t period = pdMS_TO_TICKS(1000 / kIMU_ODR_HZ);  // 1000/200 = 5 ms
```
`pdMS_TO_TICKS(5) = 5 * 100 / 1000 = 0`. FreeRTOS asserts that
`xTimeIncrement > 0`, so the task panics immediately.

**Fix (pick one — option A recommended)**

| Option | Change | Trade-off |
|--------|--------|-----------|
| **A – raise tick rate** | In `sdkconfig.defaults`: `CONFIG_FREERTOS_HZ=1000` | 1 ms ticks, perfectly standard on ESP32-S3. Marginal CPU overhead. |
| **B – guard the period** | `const TickType_t period = std::max((TickType_t)1, pdMS_TO_TICKS(1000 / kIMU_ODR_HZ));` | Safe but caps effective IMU rate at tick rate. At 100 Hz ticks, IMU runs at 100 Hz, not 200 Hz. |
| **C – lower IMU ODR** | Set `kIMU_ODR_HZ = 100` | 100 Hz is still fine for gesture recognition (see §3 below). |

**Recommendation:** Option **A** — set `CONFIG_FREERTOS_HZ=1000`. This also improves timing
resolution for the ADC tasks and `vTaskDelay`-based scheduling everywhere.

### P0-2 · IDLE1 watchdog timeout (CPU 1 starvation)

**Symptom**
```
E (23300) task_wdt: Task watchdog got triggered. The following tasks/users
  did not reset the watchdog in time:
 - IDLE1 (CPU 1)
```

**Root cause**
Core 1 hosts **six tasks**:

| Task | Priority | Count |
|------|----------|-------|
| `ads_mixed` (per ADS1015) | `configMAX_PRIORITIES - 1` (24) | ×4 |
| `sd` (SD writer) | 5 | ×1 |
| `imu` | 4 | ×1 |
| **IDLE1** | **0** | ×1 |

The four ADC tasks run at the highest possible priority. Each blocks on a binary
semaphore with a 50 ms timeout. Under normal conditions this is fine — they spend
most of their time blocked. However:

1. When all 4 ADCs produce conversions rapidly (2400 SPS × 4 chips ≈ 9600
   ISR events/s), the semaphore is signalled every ~100 µs. Four tasks at
   max priority service these back-to-back with I2C transactions (~100 µs
   each), leaving **zero idle time** on core 1 for seconds at a time.
2. The 5-second watchdog window (`CONFIG_ESP_TASK_WDT_TIMEOUT_S=5`) expires
   before IDLE1 can run.
3. This is exacerbated when the IMU task panics (P0-1), as the crash handler
   itself consumes time.

**Fix**

1. **Insert a `taskYIELD()` or brief `vTaskDelay(1)` every N conversions** in
   `mixedContinuousTask` to let IDLE1 run. For example, yield once every 100
   samples.
2. **Lower ADC task priority** to `configMAX_PRIORITIES - 2` (23). The ISR
   itself runs at interrupt level; the task just needs to be higher than SD/IMU,
   not at absolute max. This gives the scheduler more room.
3. Alternatively, **feed the watchdog** from the ADC task directly with
   `esp_task_wdt_reset()`, but this masks the starvation rather than fixing it.

### P0-3 · Channel fast/slow classification bug

**Symptom**
D-lines show data only in `adc1_0` and `adc1_1` columns; channels 2 and 3 are
always `0` even in All mode. The envelope data silently leaks into the raw queue.

**Root cause**
Channel assignment is non-contiguous (`kEMG0=0, kEMG1=2, kENV0=1, kENV1=3`),
but the fast/slow check uses a range comparison:
```cpp
bool fast = (ch <= kEMG1);   // ch <= 2  →  channels 0, 1, 2 are "fast"
```
Channel 1 (`kENV0`, envelope) is **incorrectly classified as fast**; channel 2
(`kEMG1`, raw EMG) is fast but also catches channel 1.

This causes:
- Envelope samples on ch 1 are put into `rawQ` instead of `envQ`.
- The UART D-line displays envelope samples in the wrong column.
- The SD R.bin file contains envelope data mislabeled as raw EMG.

**Fix**
Replace the range check with an explicit membership test:
```cpp
bool fast = (ch == kEMG0 || ch == kEMG1);  // channels 0 and 2
```
This affects `onSample()`, the UART header loop, and the UART data loop —
three locations in `main.cpp`.

### P0-4 · `recStart` torn reads (non-atomic 64-bit on 32-bit arch)

`recStart` is `int64_t`, read from ISR-context callbacks on core 1 and written
from `app_main` on core 0. On the Xtensa 32-bit architecture, a 64-bit load is
**two separate 32-bit loads** — an interrupt between them produces a torn value,
causing timestamps to spike by up to ~4295 seconds.

**Fix**
Use a 32-bit microsecond offset (`uint32_t`, wraps every ~71 minutes — fine for
recording sessions), or use `__atomic_store_n` / `__atomic_load_n` with a
`int64_t` and accept the cost of a critical section.

---

## P1 — DATA QUALITY (fix before trusting collected datasets)

### P1-1 · MUX bleed — first sample after channel switch is stale

The ADS1015 datasheet (§9.3.3) states that after a MUX change in continuous
mode, the **first conversion still contains data from the previous channel**.
The current driver (`mixedContinuousTask`) reads every conversion and delivers
it with the *new* channel label. This means ~50% of samples carry data from the
wrong channel.

**Fix**
After writing a new MUX config, set a flag. On the next DRDY, read and **discard**
the result, then consume from the second conversion onward. This halves the
effective per-channel rate but guarantees correct data.

### P1-2 · Timestamps reflect task wake time, not conversion time

`onSample()` calls `esp_timer_get_time()` after the ISR → semaphore → task
wake chain. FreeRTOS scheduling adds variable latency (tens to hundreds of µs).

**Fix**
Capture the timestamp inside the ISR (IRAM-safe `esp_timer_get_time()`) and
store it in a volatile member of ADS1015. Pass that timestamp to the callback
instead of calling `esp_timer_get_time()` in `onSample()`.

### P1-3 · No sample drop counter

`xQueueSend(..., 0)` silently drops samples when the queue is full. There is no
way to know post-hoc whether data was lost.

**Fix**
Add `static volatile uint32_t rawDrops, envDrops, imuDrops;` counters.
Increment on `xQueueSend` failure. Report in `#STATUS` responses and
periodically via UART.

### P1-4 · No cross-ADC sync event

Each ADC's internal oscillator drifts independently (~1% tolerance). Over a
5-minute recording, ADC0 and ADC3 can diverge by ~300 ms.

**Fix (medium effort)**
At recording start, write the config register to all 4 ADCs as close together
as possible (bus 0 pair, then bus 1 pair — ~200 µs gap). Record a "sync
timestamp" in the master header. This doesn't eliminate drift but aligns the
starting phase.

**Fix (post-hoc)**
On the Python side, resample all channels to a common uniform grid using their
per-sample timestamps. This requires P1-2 to be fixed first.

---

## P2 — ROBUSTNESS

### P2-1 · SD-fail graceful degradation

Log output shows `SD=FAIL` but recording proceeds normally (SD writer task
spins on `if (!recording)` / `if (!filesOpen)` with no data to write). This is
fine, but:
- The LED should indicate the SD failure (currently: red at init, then green
  when recording starts regardless of SD state).
- The `#STATUS` response should include SD state.
- The SD writer task should **not be created at all** when `sdOK == false`
  (saves 8 KB stack + scheduler overhead).

### P2-2 · Semaphore timeout logging

The 50 ms timeout in `mixedContinuousTask` / `continuousTask` silently absorbs
missed DRDY edges. Add a periodic warning when the timeout fires (e.g., once
per second) so the user can detect hardware issues.

### P2-3 · File transfer blocks main loop

The `G` (file transfer) command does synchronous `f_read` + `uart_write_bytes`
on the main loop. During a long transfer, UART command processing and reed
switch polling stall.

**Fix**
Either reject `G` while recording, or move file transfer to a dedicated
temporary task.

---

## P3 — SIGNAL PROCESSING & PROTOCOL IMPROVEMENTS

### P3-1 · IMU sample rate selection

200 Hz is appropriate for gesture recognition:

| Signal of interest | Bandwidth | Required ODR (Nyquist) |
|--------------------|-----------|------------------------|
| Hand/wrist gestures | <20 Hz | ≥40 Hz |
| Wrist orientation | <30 Hz | ≥60 Hz |
| Tremor / vibration | 30–50 Hz | ≥100 Hz |

200 Hz provides comfortable headroom. Reducing to 100 Hz would halve SPI bus
traffic and IMU queue usage with negligible information loss for Ninapro-style
protocols. This is a free optimisation once P0-1 is resolved.

### P3-2 · Add sequence numbers to Sample struct

Add a `uint8_t seq` (wrapping 0–255) per ADC to each Sample. This lets
post-processing reconstruct exact ordering and detect drops independently of
timestamp accuracy.

**Impact:** Sample grows from 8→9 bytes; adjust SD batching accordingly or pad
to 10 bytes.

### P3-3 · Resampling in Python companion

The IA-Arm datalogger's `SerialDataSource` reads UART D-lines at ~50 Hz
snapshot rate, **not** the actual per-sample EMG rate (~9200 raw Sa/s in All
mode). This is a visualization aid, not a dataset.

For Ninapro-grade datasets, the workflow should be:
1. Record → SD binary files (R.bin, E.bin, I.bin, M.bin)
2. Download via `G` command or SD card reader
3. Parse with `_parse_sd_session()` (already implemented)
4. Resample all channels to uniform grids (to be implemented)

### P3-4 · Add `pyserial` to requirements.txt

`SerialDataSource` does `import serial` but `pyserial` is not listed in
`requirements.txt`.

---

## Execution order

```
Phase 0  (blocking — do first)
  P0-1  Fix xTaskDelayUntil crash         → sdkconfig.defaults
  P0-2  Fix IDLE1 watchdog starvation     → ADS1015 driver or main.cpp
  P0-3  Fix fast/slow channel classification  → main.cpp (3 locations)
  P0-4  Fix recStart atomicity            → main.cpp

Phase 1  (data quality — before real recordings)
  P1-1  MUX bleed discard                 → ADS1015.cpp
  P1-2  ISR-time timestamps               → ADS1015.cpp + main.cpp
  P1-3  Drop counters                     → main.cpp
  P1-4  Cross-ADC sync                    → main.cpp

Phase 2  (robustness)
  P2-1  SD-fail graceful handling          → main.cpp
  P2-2  Semaphore timeout logging          → ADS1015.cpp
  P2-3  File transfer isolation            → main.cpp

Phase 3  (enhancements)
  P3-1  IMU ODR tuning                    → main.cpp
  P3-2  Sequence numbers                  → main.cpp + structs
  P3-3  Python resampling pipeline        → datalogger_app.py
  P3-4  pyserial in requirements.txt      → requirements.txt
```
