# EMG8 Bracelet — Improvements Work-Plan

> Originally generated 2026-04-13 from firmware review + crash log analysis.
> Updated 2026-07-20 after a second review (fidelity/timing/streaming focus) and
> a round of fixes on branch `local-libs`. Resolved items are kept for history —
> the commit that fixed each one is linked instead of deleting the entry.
> Refer to `git log` on `src/main.cpp` and the ADS1015 / I2C drivers for detail.

---

## Resolved

### From the April review (P0 — crashes & watchdog)

- **`xTaskDelayUntil` assert / panic reboot** — `CONFIG_FREERTOS_HZ` is `1000` in both sdkconfig files (`sdkconfig.defaults`, `sdkconfig.esp32-s3-devkitc-1`). Fixed prior to the July review.
- **IDLE1 watchdog timeout (core 1 starvation)** — superseded by the July per-bus rework: acquisition no longer runs 4 max-priority polling tasks. See "I2C driver migration" below.
- **Channel fast/slow classification bug** (`ch <= kEMG1` range check) — fixed prior to the July review; `onSample()` now uses `ch == kEMG0 || ch == kEMG1`.
- **`recStart` torn reads** — fixed prior to the July review via `std::atomic<int64_t> recStart`.

### From the July 2026 review

- **Mixed-rate scheduler lockup** (`1ffe73c`) — `nextMixedChannel()` compared `fastCycleCount_ % divider`, which stayed true forever once the divider was reached (the counter only advances on fast cycles), locking All-mode sampling into envelope-only after ~17 ms. Replaced with a per-slow-channel next-due counter in [ADS1015.cpp](lib/sensors-library/src/ADS1015.cpp).
- **SD file truncation on pause/resume or mode switch** (`8cddee2`) — fixed-name files opened with `FA_CREATE_ALWAYS` destroyed all prior data on every restart within a boot session. Each recording start now opens a fresh indexed file set (`R000.bin`, `R001.bin`, …); `f_open` failures are checked and reported (`#ERR:SD_OPEN`) instead of writing through invalid handles; the writer drains all queues before closing so the last ~0.8 s of samples aren't dropped on stop.
- **Mode-switch state machine** (`6216cd3`) — switching modes mid-recording left `recording = true` through the countdown, and permanently if the countdown was aborted (ADCs stopped, LED still green, UART still streaming frozen values). The switch path now stops cleanly first.
- **ADC presence detection non-functional** (`1db56f3`) — `checkForDevice()` probed all four possible I2C addresses and errored whenever any was unpopulated (always, since each bus only has two ADCs); `main.cpp` then inverted the `esp_err_t` result, treating that error as "found." `adcOK` was `true` with no ADCs connected. Now uses an address-only bus probe (`I2C::probe()`) and correct `== ESP_OK` test.
- **ISR-time timestamps** (`66045cb`) — samples were timestamped in the conversion callback, after the ISR → semaphore → task-wake chain, so FreeRTOS scheduling jitter (tens to hundreds of µs) landed on the time axis. The DRDY ISR now captures the µs timestamp directly; it's threaded through the (now 4-argument) `ConversionCallback`.
- **Build performance** (`675e99b`) — `-Og` → `-O2`, CPU 160 → 240 MHz, flash size corrected 2 MB → 8 MB (the N8 module's actual size; this was also silently truncating the effective app partition).
- **`ADXL345::isIntFlagUp` uninitialized read** (`154adbc`) — `-O2`'s stricter analysis caught a real bug: `flags` was read without being set when `getInterruptReason()` failed over I2C. Not on the EMG hot path, but a real bug in a vendored library.
- **I2C driver migration** (`c35c779`) — `I2C` (sensors-library) moved from the legacy `i2c_master_cmd_begin` API (command-link alloc/free per transaction) to the new `i2c_master` driver with cached per-address device handles; added `I2C::probe()`. This was the actual throughput ceiling: 4 ADCs × 2400 SPS ≈ 9600 conversions/s exceeded what one polling task on the legacy driver could service, causing silent rate reduction (not visible in the drop counters, since the ADS1015 just falls behind rather than filling a queue).
- **Single polling ADC task → per-bus event-driven tasks** (`c35c779`) — replaced the one task doing non-blocking polls across all 4 ADCs (1 ms worst-case added latency per poll, and it serialized both I2C buses through one task) with `adc0`/`adc1`, one per bus, each blocking on a DRDY event queue fed from the ALERT ISR. Buses now transact concurrently; idle tasks sleep instead of polling. This is also what resolved the IDLE1 starvation risk from the April review — the four max-priority polling tasks are gone.
- **No way to verify per-channel rate symmetry** — `getSampleCount()`/`resetSampleCounts()` added to `ADS1015`; `main.cpp` prints `#CNT:<adc>,<c0>,<c1>,<c2>,<c3>` on every stop/pause. Verify this on hardware: after a ~30 s All-mode recording, fast-channel counts on one ADC should match within ±1, envelope channels likewise.
- **WiFi/UDP streaming feasibility → implemented** (`1577192`) — SoftAP + UDP added as an optional, off-by-default feature (`net_stream.cpp/hpp`); see the README's "WiFi / UDP Streaming" section. SD remains the ground-truth record; the network path is non-blocking and fan-out from `onSample()`/`imuTask` so a WiFi stall cannot affect acquisition or SD logging.
- **MUX bleed — samples attributed to the wrong channel** — confirmed on real hardware 2026-07-20: raw EMG columns intermittently read ~0 (the true value of the envelope channel converted immediately before them) while envelope columns intermittently read raw-magnitude values (~600), even though `#CNT` showed correct, symmetric per-channel counts (20:1 fast:slow ratio, matching across all 4 ADCs) — proving the *scheduling* was correct and the *bug was in which channel a given conversion actually reflects*. Per the ADS1015 datasheet (§9.3.3), the first conversion completed after a MUX switch still reflects the *previous* channel; because this round-robin's fast channels always alternate (`ch0, ch2, ch0, ch2, ...`) and slow channels are injected as one-off substitutions, **the MUX switches on effectively every service call**, so effectively every sample — not "a fraction" as originally scoped here — was one channel behind its label. Fixed: a `settling_` flag set right after every `writeConfig()` mux switch (including the very first conversion after `start*()`, since the chip's prior state doesn't match either); the next DRDY-triggered conversion is read (to clear the ALERT condition) but discarded — not attributed, not counted, not delivered to the callback. Applied to all three service paths (`serviceConversion()`, `mixedContinuousTask()`, `continuousTask()`) for consistency, though only `serviceConversion()` is on the bracelet's live path. `getEffectiveSampleRate()` updated to reflect the resulting ~2× throughput cost of any multi-channel round-robin (single-channel configs are unaffected — the MUX never moves, so nothing is discarded).
  **Verify on hardware:** re-run the same recording and check `#CNT` — expect roughly *half* the previous per-channel counts (same duration, one discard per kept sample now) but the same 20:1 fast:slow ratio and cross-ADC symmetry as before; and check that D-line/UDP/SD raw columns now hover consistently near the true signal level instead of toggling between that and ~0.

---

## Open

### No cross-ADC sync event

Each ADS1015's internal oscillator drifts independently (~1% tolerance); over a multi-minute recording, ADC0 and ADC3 timestamps (though each individually accurate post-ISR-timestamp fix) drift apart. Two options, not mutually exclusive:
- **At recording start:** issue the first `writeConfig()` to all 4 ADCs as close together as possible (bus 0 pair, then bus 1 pair) and record a per-ADC "first sample" timestamp in the master header, so post-hoc alignment has a known starting offset.
- **Post-hoc (preferred, now that P1-2/timestamps are fixed):** resample all channels to a common uniform grid on the Python side using their per-sample ISR timestamps.

### Semaphore timeout logging

The 50 ms DRDY semaphore timeout in `serviceConversion()`/`mixedContinuousTask()`/`continuousTask()` silently absorbs missed edges (e.g. a wiring fault or ADC lockup). Add a periodic warning (e.g. once per second) when the timeout fires repeatedly, so hardware issues are visible instead of just showing up as reduced `#CNT` numbers after the fact.

### File transfer (`G` command) blocks the main loop

`processUartLine()`'s `G<path>` handling does synchronous `f_read` + `uart_write_bytes` directly on the task that also polls UART commands and the reed switch. During a large file transfer, mode-switch commands and the reed switch stall. Low urgency since `G` is already rejected while recording (`#ERR:BUSY`), but worth moving to a dedicated task if file transfer ever needs to run concurrently with anything else (e.g. while WiFi streaming is active but not recording).

### Sequence numbers on SD sample records

Superseded in intent by the `#CNT` per-channel counters and the UDP stream's per-type sequence numbers, but those only cover live sessions — an SD file examined after the fact still has no explicit way to distinguish "no drop" from "drop that happened to not show up as a timestamp gap." If per-record provenance in the `.bin` files themselves ever becomes necessary (e.g. for automated QA on datasets), add a wrapping `uint8_t seq` per ADC/channel; `Sample` would need to grow from 8 to a padded 10 bytes, which changes the on-disk format (bump the master header version).

### IMU output data rate

Currently 200 Hz — comfortable headroom over the ≥40–100 Hz needed for gesture/tremor bandwidth (Nyquist). Dropping to 100 Hz would roughly halve SPI traffic and IMU queue usage with negligible information loss for Ninapro-style protocols, if IMU-side bus load ever becomes a constraint. Not currently a bottleneck — no action needed unless profiling says otherwise.

### Python companion tooling (outside this repo)

Not part of this firmware repository, but noted for whoever owns the datalogger:
- Any tool that lists/downloads SD files needs to glob `R<nnn>.bin` / `E<nnn>.bin` / `I<nnn>.bin` / `M<nnn>.bin` (indexed per recording) instead of the old fixed `R.bin` names.
- The master header is now 32 bytes (version 4), not the 16-byte layout some older parsing code may assume — see the README's SD Binary Format section for the current layout.
- UART `D` lines are a ~50 Hz visualization snapshot, not the dataset — the SD `.bin` files are the full-rate record. The new UDP stream (`W1`) is a second live view at full rate, still not a replacement for SD as ground truth.
- `pyserial` should be listed in whatever `requirements.txt` the companion app uses, if not already.

---

## Suggested order for the open items

1. Cross-ADC sync (post-hoc resampling approach) — needed before trusting multi-ADC time alignment in any downstream ML pipeline.
2. Semaphore timeout logging — cheap observability win.
3. File transfer isolation — only matters once WiFi streaming and file transfer might overlap in a real workflow.
4. SD sequence numbers — only if a concrete need for it shows up; otherwise skip.
