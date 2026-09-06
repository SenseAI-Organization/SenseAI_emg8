# Acquisition review (in progress, 2026-09-05)

## Scope

Target: at least 1000 acquired samples/s on each of eight raw channels in All
mode, with envelope division by 20 and 200 Hz IMU. The attached ADC inputs are
floating: this bench checks timing and delivery, not analog signal quality.
The faulty SD card is excluded from hardware tests. Sensor mode repair is out
of scope; the desktop monitor must remain unchanged.

Preserve single-shot conversion/channel attribution, gains, the corrected
per-ADC map, wire and file layouts, SPI2/3 ownership fix, commands, countdown,
button/reed behavior, and companion UART triggers.

| ADC | I2C bus | Raw channels | Envelope channels |
|---|---|---|---|
| 1 | 0 | 0, 3 | 1, 2 |
| 2 | 0 | 1, 3 | 0, 2 |
| 3 | 1 | 0, 3 | 1, 2 |
| 4 | 1 | 1, 3 | 0, 2 |

## Current data path

ADCs use single-shot conversions at nominal 3300 SPS. A DRDY interrupt captures
a timestamp, gives that ADC's semaphore and queues its index. Two high-priority
workers on core 1 each service one bus. Service reads the conversion, publishes
it, then triggers the next channel. Raw channels alternate; each envelope
channel is inserted once per 20 raw cycles. Raw-only and envelope-only modes
sample their selected group at full speed.

The callback sends timestamped 8-byte samples to storage queues and, when
subscribed, network queues. UART displays latest values; its line rate is not
the acquisition rate. UDP batches raw/envelope/IMU into version-1 datagrams.
Packet sequences detect reception loss; ADC counters count results read.

At the target rates, storage payload is about 71,200 bytes/s: 64,000 raw,
3,200 envelope and 4,000 IMU, excluding headers and labels. Average bandwidth
alone cannot establish SD reliability; latency and error handling matter too.

## Ready-pin routing: confirmed mismatch on the attached setup

An isolated boot probe triggered one ADC at a time while polling all four
ready GPIOs, without acquisition workers or ISR handlers. Three trials per ADC
agreed, with successful I2C operations and config readback C3C0:

| ADC | Previous ready GPIO | Measured ready GPIO | Trigger-to-edge |
|---|---:|---:|---:|
| 1 (bus 0, 0x48) | 40 | 15 | 389-395 us |
| 2 (bus 0, 0x49) | 41 | 42 | 391-396 us |
| 3 (bus 1, 0x48) | 42 | 41 | 382-387 us |
| 4 (bus 1, 0x49) | 15 | 40 | 378-388 us |

The map was reversed: each worker used another ADC's ready signal. This also
means baseline software read counts are not proof of equally many distinct,
completed conversions. Preserve that qualification when interpreting throughput.
The correct mapping has passed the same isolated probe and is being tested
in acquisition. No scheduler optimization has been applied yet.

Artifacts: benchmarks/rdy-probe and benchmarks/rdy-corrected. The corrected
map is temporarily limited to EMG8_ADC_TIMING builds pending confirmation
that the bench wiring matches the actual bracelet. Normal firmware retains
its old map during that clarification.

## Other findings and next steps

| Finding | Consequence | Next step |
|---|---|---|
| SD queues fill without a writer when SD is unavailable | Wasted work and misleading storage-drop counters | Gate inactive storage enqueues; measure separately |
| Callback precedes the next conversion trigger | Publication extends every cycle | Measure, then test triggering before publication |
| I2C and GPIO interrupts are allocated on core 0; workers run on core 1 | Synchronous operations require cross-core wakeups | Measure delays and compare core-1 initialization |
| Main starts/stops ADCs while workers can service them | Configuration can overlap in-flight service | Send lifecycle commands through each owning worker |
| Recovery only runs when the whole bus queue times out | A stalled ADC may never recover while its partner runs | Check individual deadlines during partner activity |
| DRDY queue-send failures were ignored | Lost wakeups were invisible | Diagnostic build counts queue overflow |
| Wi-Fi stop delays 20 ms before resetting shared resources | Time elapsed does not prove the worker stopped using them | Add explicit worker acknowledgment |
| SD write sizes/results and sync errors are ignored | A recording may appear successful after a write failure | Reviewed; writer repair and physical validation deferred |
| SD sync counts 25 productive iterations | Claimed 500 ms interval varies with traffic | Use elapsed time in the SD follow-up |
| SD closes asynchronously after acquisition stops | Rapid restart can overlap the preceding recording's drain | Validate recording-boundary handshakes with a working card |
| SD header omits the corrected channel map | Old/new recordings cannot explain their map | Consider reserved-byte metadata separately |

The shared GPIO service is installed first by button initialization. Its
comment saying the ADC drivers already installed it is stale. Preserve the
button handler when changing interrupt placement.

The existing 1 MHz I2C operation needs protocol/electrical qualification: the
[ADS1015 datasheet](https://www.ti.com/lit/ds/symlink/ads1015.pdf) specifies a
high-speed entry sequence above fast-mode speeds. No clock increase is planned.
Successful transactions alone do not qualify the bus.

## Evidence

Original artifacts/configuration/hashes are preserved under
`benchmarks/baseline-d4ca979/firmware`; captured boot ELF hash matches the backup.
Its app version is f815a21-dirty; the directory label is not its build identity.

The baseline completed 32 runs (eight per condition, 60 seconds each).
Per-channel raw averages varied substantially without a firmware change:

| Condition | Raw channel range, Hz | Median of each run's slowest raw channel, Hz | Minimum ADC delivery |
|---|---:|---:|---:|
| Wi-Fi off | 460.3-640.6 | 460.9 | N/A |
| Wi-Fi on, no subscriber | 465.5-922.1 | 466.0 | N/A |
| UDP | 442.4-675.2 | 442.9 | 97.826% |
| UDP, UART quiet | 443.2-855.3 | 444.0 | 98.541% |

Ranges combine all raw channels and repetitions, not instantaneous peaks.
These are firmware counts divided by the host observation interval, with small
serial/start/stop timing uncertainty. All-mode envelope counts retain /20.

No I2C errors occurred. Seven retriggers occurred across the 32 runs.
Network queue drops were zero; quiet-06 reported 20 socket-send errors and
udp-08 reported 88. Other loss-bearing runs had zero socket errors, so all
reception loss cannot be attributed to a single mechanism. Wi-Fi signal was
99% (-29 dBm) during a later capture; that snapshot does not rule out interference.
Full run summaries and aggregate.json are under benchmarks/baseline-d4ca979.

Fast acquisition occurred without a UDP subscriber; UART silence is not a
reliable fix. Acquisition speed and network delivery must be assessed separately.
No baseline condition reached 1000 Hz on every raw channel.

Commit a363b15 adds resumable benchmarks. Commit b3f3612 adds opt-in timing
diagnostics and an SD-disabled build. Both firmware configurations build and
seven parser fixtures pass. These are measurement checkpoints; the 1000 Hz
target has not yet been achieved.

Acceptance: eight 60-second repeats per condition and a 15-minute All-mode UDP
run. Check each raw channel in complete 10-second windows, envelope ratio,
I2C errors, recovery events, resets, and UDP delivery. Keep device acquisition
and host reception measurements distinct.
