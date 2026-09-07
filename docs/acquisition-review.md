# Acquisition review (in progress, updated 2026-09-07)

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
workers on core 1 each service one bus. Service reads the conversion, triggers the next channel, then publishes the
completed result while that next conversion runs. Raw channels alternate; each envelope
channel is inserted once per 20 raw cycles. Raw-only and envelope-only modes
sample their selected group at full speed.

The callback sends timestamped 8-byte samples to storage queues when SD is
available and, when
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
The corrected mapping passed the isolated probe and subsequent acquisition
captures. Lifecycle ownership, ready validation and GPIO filtering now protect
the measured bench path; see the later checkpoints below.

Artifacts: benchmarks/rdy-probe and benchmarks/rdy-corrected. The corrected
map is temporarily limited to EMG8_ADC_TIMING / EMG8_BENCH_RDY_MAP builds
pending confirmation
that the bench wiring matches the actual bracelet. Normal firmware retains
its old map during that clarification.

## Other findings and next steps

| Finding | Consequence | Next step |
|---|---|---|
| SD queues fill without a writer when SD is unavailable | Wasted work and misleading storage-drop counters | Fixed and measured in 9844004 |
| Callback preceded the next conversion trigger | Publication extended every cycle | Trigger first after read; verified in 36bf23c |
| I2C/GPIO interrupts core 0, workers core 1 | Possible cross-core wakeup cost | Core-1 and split-core trials did not improve the slowest UDP channels; original layout retained |
| Main previously started/stopped ADCs while workers could service them | Configuration could overlap in-flight service | Worker-owned lifecycle with acknowledgment in 2313bd4 |
| Recovery only runs when the whole bus queue times out | A stalled ADC may never recover while its partner runs | Fixed: periodic per-chip checks; recover an existing valid completion before re-arming |
| DRDY queue-send failures were ignored | Lost wakeups were invisible | Diagnostic build counts queue overflow |
| Wi-Fi stop delays 20 ms before resetting shared resources | Time elapsed does not prove the worker stopped using them | Implemented and verified with a paused in-flight iteration; cleanup waits for acknowledgment |
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

## Later measured checkpoints (2026-09-07)

Ready validation exposed thousands of ADC4 interrupts with its pin inactive in
UDP runs. The ESP32-S3 two-sample-clock hardware GPIO glitch filter eliminated
these in the complete follow-up captures, and accepted ready timing returned
to the isolated-probe range. Physical cause is not established. A third long
filter run lost part of its serial diagnostic output; its missing ADC4 event
count is unknown. Benchmark parsing now rejects incomplete diagnostic captures.

The boot-only RMT clock probe measured regular 550 ns low and 550 ns high SCL
periods on both buses (~909 kHz). Clocking spans ~40-51 us per write/read. This
shows substantial additional time in the software transfer path; it does not
qualify electrical rise times or the TI high-speed entry protocol.

Latest measured trigger-before-publication build: ~904 Hz/raw off, ~845 Hz/raw
UDP, 30 s each, complete diagnostics, zero invalid ready events/I2C errors/
retriggers, and 100% ADC UDP delivery. Target is still unmet. Storage availability
gating and ADC lifecycle fixes are retained; SD writer physical validation,
and acknowledged network shutdown remain pending. Individual stall deadlines
are now checked during partner activity; SD-free fault injection verified both
lost-ready re-arm and lost-queue completion recovery on both buses (about 5-6 ms).
The optional static-buffer legacy-I2C comparison reached ~934-937 Hz/raw over
UDP with timing diagnostics. A control without detailed timing reached
~948-950 Hz/raw, 100% delivery, no I2C errors/retriggers in 60 s; every complete
10 s channel window was below 1000 Hz (947.3-949.9). The clock remains unchanged.
Core-1 placement also regressed this driver and was reverted. The alternative
stays in explicitly named bench environments; normal firmware retains i2c_master.

## Combined transfers and recovery checkpoint

Periodic per-ADC deadlines are verified in fe1b30e, including lost semaphore
and lost queue notifications while partners remain active. Pending valid
completions are serviced before re-arming. The optional legacy bench path now
reads the old result and writes the next configuration in one command list.
All public sample fields and channel scheduling remain the same.

The first 30 s comparison reached ~1017 Hz/raw off and ~1005-1006 Hz/raw UDP,
with every complete 10 s UDP raw window above 1000 Hz. Envelopes remain /20
and IMU 200 Hz. All diagnostics complete, zero invalid ready events/I2C errors/
retriggers, 100% ADC delivery, no gaps. Artifacts: benchmarks/combined.
This is preliminary evidence; full repeated and long-run acceptance is pending.

Simulated return errors before/after accepted commands and after a 20 ms delay
verified conservative suppression of uncertain samples. Recovery waits a full
5 ms after error return and re-arms the named channel without advancing the
schedule again. Six deliberate errors caused exactly six clean recoveries;
counts confirm six publications were suppressed. Artifacts: benchmarks/combined-fault.
Temporary fault hooks were removed before final builds.

The clean no-timing build subsequently passed three 60 s UDP runs: every raw
channel 1010.27-1010.83 Hz, all 120 complete 10 s raw windows 1010.0-1011.2 Hz,
100% ADC delivery, zero gaps/I2C errors/retriggers. Envelopes remain /20 and
IMU ~200 Hz. These throughput rates do not imply uniformly spaced samples.
Full matrix/soak acceptance and SD validation remain pending.

Three repeats of all four conditions kept raw rates/windows above 1000 Hz.
One quiet capture exposed five failed send calls that discarded five packets
(364 ADC samples). The sender now retains rejected batches for retry; a
targeted test recovered nine injected rejections with 100% delivery and no gaps.
A direct socket-close marker verified W0 waits for an in-flight worker before
cleanup. Full acceptance will use the clean build containing these fixes.

## Clean sender validation checkpoint

The combined SD-free throughput build with retry/acknowledged shutdown passed
a 15-minute All UDP run: 1009.52-1010.02 acquired Hz/raw, every complete 10 s
raw window above 1000 Hz (minimum 1004.2), envelopes /20, IMU approximately
200 Hz. ADC delivery was 99.9724%, exceeding the 99.5% threshold but not lossless.
No I2C errors/retriggers/resets or network queue drops occurred. There were
2283 rejected send attempts and 16 gaps among successfully submitted packets;
the errno and downstream loss location remain unresolved. See WORKLOG.md and
benchmarks/net-retry-clean/soak-15m for exact counts and qualifications.

Raw/Env 60 s mode checks and UART lifecycle transitions passed. One clean 60 s
run per off/no-subscriber/UDP/quiet condition passed; seven more repeats per
condition remain for the full matrix. Physical SD and button/reed/companion
tests were not performed in this checkpoint. The optimized path remains a
bench configuration pending normal deployment decisions.
