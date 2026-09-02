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

**Sin commitear:** nothing — Daniel committed this work as `442d143`, then the
mutex revert as `9c501d1`.

---

## 2026-08-27 (end of day) — Mutex reverted, acquisition verified healthy

**Hecho:**

- **Reverted the ADC mutex and ALERT/RDY re-arm** (`9c501d1`, Daniel). They were
  added to explain the sensor-test stall, fixed nothing, and were suspected of
  destabilising sampling. The driver is back to the `2ad6518` state — the
  firmware that was on the device when the day's testing began.
- **Acquisition verified healthy on hardware by Daniel**: all 8 raw and all 8
  envelope channels reading correctly in All mode. Independently corroborated
  by a 10 s `#CNT` capture: 5541–5543 fast / 277 slow per ADC (exactly the 20:1
  divider ratio), symmetric across all four, with **zero** retriggers and
  **zero** I2C errors. Single-shot channel attribution — the whole point of the
  2026-07-20 work — is confirmed correct.
- Measured rates supersede earlier estimates: **~554 Hz/ch raw, ~28 Hz/ch
  envelope** in All mode at 3300 SPS. README updated.
- Kept (non-sampling, hardware-verified fixes): `countdown()` now routes bytes
  through the real parser, so `L0,1` sent right after a mode command no longer
  aborts the countdown — that had been killing scripted-test starts
  deterministically; FatFs long filenames enabled; `G` resolves against the
  same base `F` lists; SD directory/open failures reported instead of silent.

**A correction worth recording:** mid-session I claimed from sensor-test stall
data that *all* acquisition might be dead and pushed for a full revert to
free-running continuous mode. That was wrong — All mode was healthy the whole
time. Daniel's instinct to protect the known-good base was right. The lesson:
a stall confined to one code path was over-generalised without testing the
path that actually mattered; the All-mode `#CNT` check that settled it took
30 seconds and should have come first.

**Pendiente:**

- **Sensor-test mode (`4` / `S<n>`) is present but NOT FUNCTIONAL.** Commands
  ack correctly; no data is delivered. Zero conversions with retriggers
  climbing ~100/s — triggers issue, conversion-complete interrupt never
  returns. Specific to the stop/start cycle `S<n>` performs; not the mutex.
  Flagged prominently in README so nobody builds against it. Either fix it or
  remove mode 4 — leaving it advertised-but-broken is the worst of both.
- `G` (SD download) is fixed but **untested since the fix**. Note the behaviour
  change: session directories are now created for real, so recordings land in
  `s_<MAC>_<epoch>/` rather than the card root.
- WiFi/UDP streaming still not exercised end-to-end.
- Open items unchanged in `improvements-workplan.md`.

**Sin commitear:** `README.md`, `WORKLOG.md`,
`lib/sensors-library/CHANGELOG.md` — documentation accuracy pass only, no code.

**Coordinación:** a second Claude session (`ia-arm-datalogger-55`) owns
`D:\PhD\Code\IA-Arm_datalogger`; this session is firmware-only from
2026-08-27 onward. Note that some of that session's firmware work
(`startRecording()` refactor, button ISR / reed guard, `sendTrigger` /
`sendStartToSlave`) was swept into commits `4015ea4` and `374ca06` here by a
`git add -A` — nothing lost, but attribution is muddled. Ping before crossing
repos.

## 2026-09-02 — Panico en la ISR de SPI durante grabacion: aislado y corregido

**Hecho:** Las grabaciones morian con un `Guru Meditation Error (LoadProhibited)`
a los 5-41 s. Reproducible con la app nueva, con la vieja y con
`serial_capture.py`, es decir independiente del anfitrion. `#BOOT:` lo confirmo
como `reset=PANIC(4)`, no brownout ni watchdog.

Backtrace decodificado contra el ELF de la build:

    bg_exit_core          spi_bus_lock.c:554     <- deref nulo, EXCVADDR 0x8
    spi_bus_lock_bg_exit  spi_bus_lock.c:778
    spi_intr              spi_master.c:1027
    _xt_lowint1 / tarea idle

**Aislamiento**, midiendo en hardware (`capture_health.py`, 100-150 s por corrida):

| modo | tarea IMU | resultado |
|---|---|---|
| 1 (All) | activa | panico a los 41 s |
| 2 (Raw) | activa | sobrevivio 97 s |
| 3 (Env) | activa | panico a los 23 s |
| 1 (All) | **desactivada** | **sobrevivio 107 s** |
| 1 (All) | activa, **con el fix** | **sobrevivio 147 s** |

Descartados por medicion, no por intuicion: fuga de memoria (heap plano en
~171 kB), contrapresion de colas (picos muy por debajo del fondo, cero
descartes), volumen de escritura en SD (el modo 2 escribe mas y no falla),
y DTR/RTS del anfitrion (falla con las lineas afirmadas y sin afirmar).

**Causa:** `SPI::write/read/transfer` envolvian un `spi_device_transmit()` —la
llamada por interrupcion— dentro de `spi_device_acquire_bus()`/`release_bus()`.
ESP-IDF espera transacciones *polling* mientras el bus esta adquirido; mezclar
las dos deja inconsistente la invariante `acquiring_dev` / `acq_dev_bg_active`
que `bg_exit_core()` lee desde la ISR. Con el IMU transfiriendo a ~100 Hz
durante toda la grabacion, tarde o temprano la ISR entraba con el lock en un
estado imposible.

**Corregido** en `cdf8608`: `spi_device_polling_transmit()`, que es el
emparejamiento documentado y ademas saca a estos dispositivos de la ruta de ISR.
Solo lo usan los drivers de IMU (ICM42605, LSM6DSOX); la SD va por `sdspi_host`
y no se toca.

**Artefactos:** `cdf8608` (fix), `65aafdf` (diagnosticos `#BOOT`/`#HEALTH` y el
interruptor de compilacion `EMG8_NO_IMU_TASK`, que ya estaban sin commitear en
el arbol). Herramientas del lado del anfitrion en
`D:\PhD\Code\IA-Arm_Monitorackend	ools\`: `capture_health.py`,
`crash_bisect.py`, `probe_device.py`.

**Pendiente:**
- Corrida larga (>10 min) para confirmar que no queda una cola mas lenta.
- El modo 2 subio de 67 a 300 de pico en la cola de crudo tras el fix: el
  camino de ADC esta rindiendo mas, sin descartes. Vale la pena volver a medir
  `#CNT` por canal contra la tasa nominal.
- UDP a tasa completa sigue sin ejercitarse contra hardware.
- `gpio_install_isr_service(502): already installed` sigue apareciendo 4 veces
  al arrancar una grabacion. Inofensivo hasta donde se ve, pero ahora que hubo
  un fallo relacionado con ISR conviene mirarlo.
- Modo 4 sigue sin entregar datos.

**Sin commitear:** nada; el arbol quedo limpio en `wireless-testing`.

**Coordinacion:** este trabajo lo hizo la sesion que tiene `IA-Arm_Monitor`,
cruzando a este repo con autorizacion explicita de Daniel y solo sobre la rama
`wireless-testing`. Sin `git add -A`: los dos commits listan sus rutas.


## 2026-09-02 (mas tarde) — UDP validado contra hardware por primera vez, y el UART fuera del camino de datos

**Hecho:** Con el panico de SPI ya corregido, se ejercito el flujo completo
serial -> UDP contra el equipo. Nunca se habia hecho.

El anfitrion hace la secuencia solo (`--source auto`): abre el puerto, `?`,
`W1`, espera `#WIFI:1`, suscribe UDP contra 192.168.4.1:3333 y promueve el
enlace a `streaming` en cuanto llega el primer paquete. Si el PC todavia no se
unio a la red del brazalete lo dice con esas palabras y reintenta cada 20 s.

**Resultado, modo 1, medido en hardware:**

|  | UART (lineas D) | UDP |
|---|---|---|
| registros crudos / 40 s | 12 632 | 128 786 |
| tasa medida | 47.6 Hz | 511 Hz agregada |
| perdida | — | 6 de 2020 paquetes (0.3 %) |

**Despues** de bajar la linea CSV a 1 Hz mientras el UDP entrega (`72d0a26`):

    trafico UART      5700 B/s  ->  138 B/s      (41x menos)
    crudo por canal    ~510 Hz  ->  ~885 Hz      (+74 %)
    envolvente          ~25 Hz  ->   ~44 Hz
    perdida de paquetes   0.3 % ->   0.05 %

El aumento de tasa no era el objetivo y es lo mas interesante: `uartTask`
estaba dejando sin CPU a las tareas de servicio de los ADC. Confirmado con los
contadores del propio equipo tras un stop limpio, sin depender del anfitrion:

    #CNT:1,63769,3188,63769,3188,0,0    razon rapido/lento exactamente 20.0
    #CNT:2,61463,3073,61463,3073,0,0    i2c_err = 0, retrig = 0 en los cuatro
    #CNT:3,61461,3073,61460,3073,0,0
    #CNT:4,63280,3164,63280,3163,0,0

499 946 conversiones en el equipo, 495 751 recibidas por el anfitrion
(99.16 %), cero desbordamientos de anillo.

**Hallazgo del protocolo:** `#NET:<ip>:<port>` NO anuncia el extremo del
brazalete, sino el del cliente que acaba de suscribirse
(`net_stream.cpp:123` imprime la direccion de origen del datagrama).
Verificado: suscribiendose desde 192.168.4.2 el equipo respondio
`#NET:192.168.4.2:54400`. Sigue sin haber forma de preguntarle al equipo cual
es su propia IP; queda como pedido P0 en `FIRMWARE-CONTRACT.md` del proyecto
del monitor.

**Pendiente:**
- El estimador de tasa del IMU reporta ~3.8 kHz con 35 % de huecos: los
  `ts_us` del IMU no se comportan como los del EMG. Revisar si
  `recordingTimestampUs()` se muestrea bien en `imuTask`. El camino de sEMG no
  se ve afectado.
- Modo estacion sigue siendo el pedido grande: el SoftAP obliga al PC a dejar
  su red, y eso impide que el enlace sea del todo automatico.
- Corrida larga (>10 min) con UDP para confirmar estabilidad.
- Modo 4 sigue sin entregar datos.

**Sin commitear:** nada.


## 2026-09-02 (correccion) — el +74 % de tasa no se sostiene

**Correccion a `72d0a26`.** Ese commit afirma que silenciar la linea CSV subio
la tasa de ~510 a ~885 Hz/canal (+74 %). La medicion era real —la confirmo el
`#CNT` del equipo— pero **no se reproduce**, y presentarla como resultado
estable fue un error.

Matriz controlada (50 s por condicion, medida siempre con `#CNT` dividido por
el tiempo entre `#REC` y `#STOP`, no con contadores del anfitrion):

| condicion | crudo Hz/canal | lineas D |
|---|---|---|
| radio apagada, D a ~48 Hz | 463.5 | 47.7 Hz |
| radio encendida, sin suscriptor, D a 1 Hz | 467.1 - 517.4 | 1.0 Hz |
| radio encendida + suscrito + transmitiendo | **727.9** | 1.0 Hz |

Lo que se puede afirmar:

- **Silenciar el UART vale ~+12 %**, no +74 %. Real, pero modesto.
- **Dos corridas de la misma condicion difieren ~2 %**, asi que la medida es
  repetible y las diferencias de arriba estan fuera del ruido.
- **Con UDP transmitiendo de verdad la tasa sube mucho (+56 % sobre la misma
  condicion sin suscriptor).** Es al reves de lo esperado: mandar mas datos
  deberia costar CPU, no regalarla. No hay explicacion todavia.
- **No es escalado de frecuencia de CPU**: `CONFIG_PM_ENABLE` no esta activo y
  la CPU esta fijada a 240 MHz.
- `i2c_err` y `retrig` en 0 en todas las condiciones.

**Siguiente diagnostico, en orden de costo:**
1. `esp_wifi_set_ps(WIFI_PS_NONE)` — una linea. El firmware no llama a
   `esp_wifi_set_ps`, asi que corre con el ahorro de energia por defecto
   (`WIFI_PS_MIN_MODEM`). Si las transiciones de modem-sleep estan retrasando
   la ISR de DRDY o la tarea de I2C, esto lo aplana.
2. Alternar w1_sub / w1_nosub varias veces para descartar deriva.
3. Contar en `adcBusTask` los despertares por cola frente a los por timeout:
   distingue "tarea sin CPU" de "flanco DRDY perdido".

Hasta que eso se entienda, la tasa util del equipo es **~460-520 Hz/canal**, y
los 885 Hz quedan como un dato aislado sin reproducir.

**Sin commitear:** nada.

## 2026-09-02 — Modo UDP-solo: `U0` / `U1`, y lo que NO arregla

**Hecho:** el equipo puede dejar de escribir por el UART entero mientras
entrega por UDP, y volver por tres caminos distintos.

`U0` / `U1` (dos bytes, sin salto de linea, como `V0` / `W1`), aceptados por
UART y por UDP. Acusan `#UART:0` / `#UART:1`.

Lo que calla `U0`:

- los ~77 `printf` de `main.cpp`, redirigidos con una sola linea
  (`static int hostPrintf(...)` + `#define printf hostPrintf`, tras los
  includes). Un gancho, ningun sitio de llamada olvidado;
- los logs del propio ESP-IDF (`esp_log_level_set("*", ESP_LOG_NONE)`), que no
  pasan por `printf`;
- el armado de la linea `D` en `uartTask`: se salta el bloque entero, no solo
  la impresion.

Lo que **no** toca es la recepcion del UART. Es deliberado: `U1` tiene que
funcionar a ciegas. Tres vias de vuelta, para que el equipo no pueda quedar
mudo y sordo a la vez:

1. el anfitrion manda `U1` solo si el UDP se cae o si va a descargar de la SD
   (`link.py`, `_restore_uart`);
2. el equipo se destapa solo tras 20 s sin datagramas del cliente
   (`kQuietWatchdogMs`) y avisa `#UART:1,watchdog`;
3. siempre queda `U1` a ciegas por el UART, o un reset por RTS.

Ademas `netStreamStop()` (o sea `W0`) devuelve el UART —sin radio no quedaria
por donde hablar— y el manejador de `G` se destapa solo, porque el cuerpo del
archivo sale por `uart_write_bytes`, que no pasa por `hostPrintf`.

El canal de comandos por UDP salio casi gratis: `pollSubscribe()` ya hacia un
`recvfrom` para aprender la direccion del cliente y tiraba el contenido. Ahora
lo que no sea el `HI` de suscripcion va a un manejador registrado
(`netSetCommandHandler`). Solo acepta `U0`/`U1`: son escrituras atomicas de una
bandera, sin efectos colaterales, y corren en la tarea de red. El resto del
juego de comandos sigue por UART, que nunca deja de escuchar.

**Verificado en hardware** (8 comprobaciones, todas pasan):
`?` contesta · `U0` acusa y luego 0 bytes en 3 s · sigue mudo ante `?` · `U1` a
ciegas lo recupera · idem con la radio encendida · **`U1` por UDP devuelve la
consola** · el watchdog de 20 s se dispara solo.

**Lo que NO arregla: la tasa.** La hipotesis era que el UART se estaba comiendo
las conversiones. No es eso, y ahora hay con que afirmarlo.

Primero, tres pares de 60 s (`#CNT`, `w1_sub` frente a `w1_quiet`):

| par | `w1_sub` | `w1_quiet` | cambio |
|---|---|---|---|
| 1 | 481.7 | **881.2** | +82.9 % |
| 2 | 460.0 | 524.5 | +14.0 % |
| 3 | 456.4 | 452.0 | −1.0 % |

`D 0.0 Hz` en las tres filas calladas: el silencio es real. Pero la dispersion
*dentro* de la condicion callada se come la diferencia.

Lo que lo cierra: **ocho corridas identicas seguidas**, misma condicion, sin
tocar nada entre una y otra:

```
808.4 · 463.8 · 461.0 · 462.2 · 455.9 · 459.8 · 460.1 · 459.3   Hz/canal
```

La tasa es **bimodal**: o ~460 Hz/canal o ~800-880 Hz/canal, un factor de casi
dos. Dentro del modo lento es repetible al ±1 % (455.9 a 463.8 en siete
corridas). Por eso cada comparacion pareada parecia concluyente: ganaba el lado
que hubiera caido en el modo rapido. El UART nunca fue la variable.

Cronologicamente, las 18 medidas de hoy —lento, RAPIDO, lento, 524, lento, lento /
RAPIDO ×4 / RAPIDO, lento ×7— no dan un patron limpio con ninguna de las
condiciones probadas. Las cuatro rapidas seguidas fueron la corrida en la que
el portatil habia perdido la asociacion y no llegaba ni un datagrama, asi que
tampoco es "mandar por UDP lo acelera".

**Un resultado util que si sale de esto:** en el modo rapido el anfitrion
recibe lo que el equipo convierte. 808.4 Hz/canal convertidos, **807.5
Hz/canal recibidos** por UDP (99.4 a 99.8 % en las dos modalidades). La ruta
UDP aguanta 800 Hz/canal de punta a punta; si el firmware llega a esa tasa de
forma estable, el anfitrion no es el cuello de botella.

**Artefactos:**
- `.pio/build/esp32-s3-devkitc-1/firmware.bin` (942 176 B), verificado con
  `grep -aoF` que contiene `#UART:0`, `#UART:1` y `#UART:1,watchdog` antes de
  grabarlo. Es lo que corre en el equipo ahora.

**Pendiente:**
- La variacion de tasa sigue sin explicacion. El siguiente diagnostico mas
  barato sigue siendo `esp_wifi_set_ps(WIFI_PS_NONE)`: el firmware nunca llama
  a `esp_wifi_set_ps`, asi que corre con `WIFI_PS_MIN_MODEM`.
- Detalle menor: `U1` deja el nivel de log global en `ESP_LOG_INFO`, asi que
  pisa el `esp_log_level_set("ICM42605", ESP_LOG_DEBUG)` de arranque. Sin
  efecto practico (ese driver no registra nada en caliente), pero esta ahi.

**Sin commitear:** nada.

## 2026-09-02 (bis) — El mapa crudo/envolvente no es el mismo en los cuatro ADC

**Hecho:** `kRawCh`/`kEnvCh` por ADC, lista de barrido armada por ADC, y
`isRawCh(id, ch)` decidiendo rapido/lento en `onSample`. La cabecera `H` gana
un sufijo `r`/`e` por columna.

Lo que suponia este firmware —crudo en ch0/ch2, envolvente en ch1/ch3, igual en
los cuatro— no es lo que hay en la placa:

| ADC | crudo | envolvente |
|---|---|---|
| 1 | 0, 3 | 1, 2 |
| 2 | 1, 3 | 0, 2 |
| 3 | 0, 3 | 1, 2 |
| 4 | 1, 3 | 0, 2 |

`ch2`/`ch3` intercambiados en los cuatro, y `ch0`/`ch1` ademas en los ADC 2 y 4
— los pares 1/3 y 2/4 estan en espejo.

**Como se midio:** captura por UDP muestra a muestra, modo 2 y modo 3 por
separado para que cada patilla corriera a tasa completa (el firmware quita el
divisor cuando no hay canales rapidos). Cada canal salio >=97.9 % en un solo
nivel y doce de dieciseis al 100 %: es estable, no una carrera de mux. La
bateria del sensor cargada, que es lo que pone el pedestal crudo en ~605.

**Por que importaba mas que una etiqueta:** con el mapa anterior, seis de los
ocho electrodos tenian su señal cruda entrando por una patilla que se
muestreaba con el divisor /20 a ~23 Hz, y su envolvente a ~460 Hz. Y cada
muestra iba al fichero equivocado en la tarjeta (R.bin frente a E.bin) y al
tipo de paquete UDP equivocado.

**Verificado tras grabar:** modo 2 entrega los ocho canales crudos a ~605 y
modo 3 los ocho de envolvente en ~0, ambos al 100 %. Antes eran 2 de 8 y 6 de
8. Tambien desaparece el 1-2 % de mezcla que se veia en el ADC 1.

**Artefactos:** `.pio/build/esp32-s3-devkitc-1/firmware.bin` (942 026 B),
verificado que contiene el formato nuevo de cabecera antes de grabarlo. Es lo
que corre en el equipo.

**Pendiente, para quien revise el esquematico:**
- Confirmar si el espejo de los ADC 2/4 es intencional (rutado) o un error, y
  si `ch2`/`ch3` estan invertidos en la placa o la convencion del firmware
  estaba equivocada desde el principio.
- **La cabecera maestra de la SD no guarda el mapa de canales.** Son 32 bytes
  con `[25-31]` reservados; escribir ahi los cuatro bytes de `kRawCh` haria que
  cada grabacion se explique sola y permitiria distinguir un fichero anterior a
  este arreglo de uno posterior. Compatible hacia atras.
- Toda grabacion anterior a este commit tiene R.bin y E.bin cruzados en seis de
  ocho electrodos. El mapa es determinista, asi que se pueden reinterpretar.
- Transitorios aislados en canales crudos (`adc4_1r`: pico a pico 301 con sd
  7.8). No los explica el cargador de la bateria: sobreviven al desenchufe.

**Sin commitear:** nada.
