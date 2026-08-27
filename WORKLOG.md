# WORKLOG — EMG8 Bracelet

Bitácora de continuidad entre sesiones. Se anexan entradas, nunca se reescribe
el historial. La memoria durable de hechos vive en `memory/`; esto guarda el
relato. Ver también `improvements-workplan.md` (estado de bugs abiertos) y
`README.md` (protocolo autoritativo).

---

## 2026-08-27 — Channel-attribution fixed (single-shot), sensor-test mode, WiFi/UDP

**Hecho:**

- **Root-caused and fixed the channel-misattribution bug** that made raw EMG
  and envelope values swap columns at random (`374ca06`). Cause was *not* a
  fixed off-by-one: on the ADS1015 the input MUX only changes *after* the
  in-progress conversion completes, so a config write lands before or after
  that internal boundary depending on I2C timing — one stale conversion or
  none, unpredictably. An intermediate "discard one after each switch" fix
  (`7807df6`) wins that race only ~half the time and was **not** sufficient.
  Now uses **single-shot triggering**: each conversion is explicitly requested
  for a named channel, so the result can only belong to it. Confirmed as TI's
  own recommendation for frequent channel swapping.
  ⚠️ **Recordings made before `374ca06` have unreliable channel attribution
  and must not be used for analysis.**
- Earlier fixes this cycle: mixed-rate scheduler lockup that froze raw EMG
  ~17 ms into All mode (`1ffe73c`); SD file truncation on every pause/resume
  (`8cddee2`); mode-switch leaving a zombie recording state (`6216cd3`);
  non-functional ADC presence detection (`1db56f3`, two bugs cancelling out);
  ISR-time sample timestamps (`66045cb`); build at -O2 / 240 MHz / 8 MB flash
  (`675e99b`).
- **I2C driver migrated** from legacy `i2c_master_cmd_begin` to the new
  `i2c_master` API with cached device handles, plus one event-driven service
  task per I2C bus instead of a single polling task (`c35c779`).
- **WiFi SoftAP + full-rate UDP streaming** added, off by default, `W1`/`W0`
  (`1577192`). SoftAP `EMG8-<MAC>` / `emg8sense` / `192.168.4.1:3333`.
  Required a custom `partitions.csv` (3 MB app) — note `board_build.partitions`
  in `platformio.ini` is what actually works; the sdkconfig `singleapp_large`
  route was silently ignored by PlatformIO's partition generator.
- **Sensor-test mode (`4`)** added for per-electrode bench checks (`2ad6518`):
  one sEMG sensor at a time at the chip's full rate (~2400 Hz vs ~1100 Hz in
  All mode), selected with `S0`–`S7`, acknowledged as `#SENSOR:<n>,<adc>,<ch>`.
- `#CNT` extended to 7 fields: per-channel counts plus per-ADC I2C-error and
  stall-retrigger counters, so bus trouble is visible instead of silent.
- **Flashed and verified**: 939936 bytes written, `Hash of data verified`,
  hard reset OK. Binary was string-checked before flashing (`#SENSOR:%u,%u,%u`
  and `s%u_adc%u_%u` both present) per the build-artifact rule.

**Artefactos:**

- `.pio/build/esp32-s3-devkitc-1/firmware.bin` — 939936 B, built 2026-08-27
  07:18, **this is the build currently on the device**.
- `.pio/build/esp32-s3-devkitc-1/partitions.bin` — custom 3 MB app partition.
- Docs: `README.md` (authoritative protocol), `improvements-workplan.md`
  (resolved vs open), `lib/sensors-library/CHANGELOG.md` (driver API changes,
  incl. the breaking `ConversionCallback` signature change).
- Cross-repo sync doc for the datalogger agent:
  `D:\PhD\Code\IA-Arm_datalogger\README-emg8.md`.

**Pendiente:**

- **Hardware verification of the single-shot fix is the top priority**: raw
  columns should sit steady near ~600 at rest and envelope near 0, with no
  swapping; `#CNT` should keep its 20:1 fast:slow ratio and cross-ADC
  symmetry, with the two new counters at or near 0.
- Sensor-test mode (`4` / `S<n>`) is flashed but **not yet hardware-tested**.
- Open items in `improvements-workplan.md`: cross-ADC sync (post-hoc
  resampling preferred), semaphore-timeout logging, file-transfer task
  isolation, SD sequence numbers.
- A sub-millisecond window exists at each `S<n>` switch where one D-line can
  read the new sensor's slot before `#SENSOR:` is emitted. Carries stale/zero
  data; deliberately left alone rather than adding complexity. Documented to
  the datalogger agent so it isn't chased as a parser bug.

**Sin commitear:**

- `src/main.cpp`, `README.md` — sensor-protocol hardening: CSV header now
  reprints when `S<n>` changes the sensor mid-recording (it previously went
  stale and mislabeled the column, same failure class as the bug fixed above);
  `#SENSOR:` also emitted on entering mode `4`; `#ERR:SENSOR` on a malformed
  selection. **Built clean but NOT flashed** — the device still runs the
  previous build, which has the stale-header bug. Daniel is reviewing before
  committing.
- This `WORKLOG.md` itself (new file).

---

## 2026-08-27 (cont.) — Sensor-test bugs found by hardware testing, fixed + flashed

Follow-up within the same day. The datalogger session exercised sensor-test
mode against real hardware and surfaced three defects in code written earlier
today. All fixed and flashed.

**Hecho:**

- **`countdown()` was corrupting multi-byte commands.** It read UART bytes and
  compared them raw, aborting on any `'0'`. So the `'0'` inside `S0` aborted
  the recording instead of selecting sensor 0 (`#CD:ABORT` + `#STOP`,
  reproduced on hardware), and — worse, found while checking that — the `'0'`
  inside `L0,1` did the same. The datalogger sends `1` then `L0,1`
  back-to-back on every scripted test start, so that path was aborting
  **deterministically**, not intermittently. Fixed by routing countdown bytes
  through `feedUartByte()`, the same parser the command loops use: `L`/`G`
  lines are consumed whole, so only a genuinely standalone `'0'` can reach the
  abort test. Scope is countdown-window-only; `F`/`G`/`?` outside that window
  were never affected.
- **Single-shot round-robin stalled permanently after stop/start churn.**
  Evidence: `#CNT` showed all-zero per-channel counts with 95–668 retriggers
  and `i2c_err = 0` on every ADC — i.e. trigger writes landing but ALERT/RDY
  never pulsing. Two causes, both fixed in `lib/sensors-library` (see its
  CHANGELOG `[0.11.0]`): `stopContinuous()` disables the comparator while the
  `ThreshLow`/`ThreshHigh` conversion-ready registers were only written once at
  boot (now rewritten on every start); and an unsynchronised race between
  `serviceConversion()`/`retriggerIfStalled()` on the ADC service task and
  `startMixedContinuousExternal()`/`stopContinuous()` called from the main
  loop on the other core (now serialized by a per-instance mutex).
- Also flashed from the earlier unflashed batch: CSV header now reprints when
  `S<n>` changes sensor mid-recording (it previously went stale and mislabeled
  the column); `#SENSOR:` emitted on entering mode `4`; `#ERR:SENSOR` on a
  malformed selection, so silence now genuinely means "not received".

**Artefactos:**

- `.pio/build/esp32-s3-devkitc-1/firmware.bin` — 940352 B, `Hash of data
  verified`, **this is now the build on the device**. String-checked before
  flashing (`#ERR:SENSOR`, `#SENSOR:%u,%u,%u`, `s%u_adc%u_%u` all present).

**Pendiente:**

- **Re-test needed to confirm the stall fix**: rerun the `S0`–`S7` sweep and
  check `#CNT` — retriggers should drop to ~0 and per-channel counts should be
  non-zero. If retriggers persist there is a third mechanism not yet found.
- Confirm scripted-test starts now reach `#REC` instead of `#CD:ABORT`.
- The single-shot channel-attribution fix itself is **still not
  hardware-verified** for the main All/Raw/Env acquisition path — that remains
  the highest-value outstanding check.
- Open items unchanged in `improvements-workplan.md`.

**Sin commitear:** everything from this entry — `src/main.cpp`,
`lib/sensors-library/{include/ADS1015.hpp,src/ADS1015.cpp,CHANGELOG.md}`,
`README.md`, `WORKLOG.md`. Daniel is reviewing before committing.

**Coordinación:** a second Claude session (`ia-arm-datalogger-55`) owns
`D:\PhD\Code\IA-Arm_datalogger`; this session is firmware-only from
2026-08-27 onward. Note that some of that session's firmware work
(`startRecording()` refactor, button ISR / reed guard, `sendTrigger` /
`sendStartToSlave`) was swept into commits `4015ea4` and `374ca06` here by a
`git add -A` — nothing lost, but attribution is muddled. Ping before crossing
repos.
