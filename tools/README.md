# Acquisition bench

Use `bench_acquisition.py` with the ADCs attached; analog sensor inputs may be
disconnected. It refuses to start if the device is recording or reports mounted
SD storage. It does not flash, reset, format, download, or modify the monitor app.
Do not use the old monitor `rate_matrix.py` to calculate current rates: its
channel-0-is-raw assumption predates this firmware's per-ADC channel map.

Example (from the firmware repository, using a Python environment with pyserial):

```text
python -B tools/test_bench_acquisition.py
python -B tools/bench_acquisition.py --condition udp --seconds 60 --wifi-profile EMG8-24EC4A368770 --output benchmarks/baseline-d4ca979/udp-01
```

The Windows Wi-Fi profile is optional and must already exist. The `8770` board
is the attached bench ESP32; the previously saved `8790` profile belongs to the
other bracelet. Ethernet should remain connected during Wi-Fi tests. Windows
may require elevated execution for `netsh wlan connect`.

Conditions are `off` (radio off), `nosub` (radio on without subscribing), `udp`,
and `quiet` (UDP with UART silenced during the measurement). Every run resets
the network session using W0/W1 as needed. UDP conditions reconnect the specified
Wi-Fi profile and require a subscription acknowledgment before acquisition.
The default mode is All; `--mode 2` and `--mode 3` exercise Raw and Env.

Each output directory must be new. Files are saved locally under the ignored
`benchmarks/` directory:

- `metadata.json`: configuration, channel map, UTC time, and timing convention.
- `serial.jsonl`: commands and non-CSV responses with host timestamps.
- `udp.bin`: complete datagrams, framed with `<QH` host nanoseconds and byte length.
- `summary.json`: every channel's counts, rates, timing quantiles, complete
  10-second reception windows, sequence losses, duplicate/reordered packets,
  I2C errors, retriggers, final status, and any failure.

`host_timed_acquired_hz` uses firmware CNT divided by the host's observed REC to
stop-command interval. It has small UART/start/stop timing uncertainty. UDP
`received_hz` uses device timestamps. It is reception throughput, not proof of
the acquisition rate if packets were lost. The delivery fraction compares all
received ADC records against all four ADC counters, including trailing batches
received after stop. The initial sequence number is a subscription boundary,
not evidence of packet loss. Out-of-order packets within the capture are kept;
duplicates are excluded from received counts.

The first firmware versions still enqueue SD samples when SD is unavailable.
Their raw/env/IMU storage-drop counters therefore rise even with perfect UDP
delivery. Preserve this evidence; do not interpret it as network loss. Later
SD-free firmware must eliminate that unused queue work.

Run each matrix condition eight times for 60 seconds, using a new output folder
per run, before making causal performance claims. A short or single run is only
a preliminary baseline. Keep the original build artifacts and full configuration
alongside the baseline, and identify the flashed build separately from the host
checkout: the existing protocol has no firmware build-ID query.

## Opt-in firmware diagnostics

Build `pio run -e esp32-s3-bench` for the attached SD-free bench. This environment
sets `EMG8_NO_SD` (skips SD initialization entirely) and `EMG8_ADC_TIMING`.
The normal environment keeps its existing SD behavior and omits timing code.
The bench still performs the original sample queue operations until a separately
measured optimization changes them.

After stop, #TIMING reports count, total/min/max microseconds, and eight histogram
bins with exclusive upper bounds 25, 50, 100, 200, 400, 800, 1600, infinity.
Metrics are config-write duration (trigger), ISR-to-service delay (wake),
conversion-register read duration (read), sample callback duration (publish),
config-write start to ready ISR (ready), and ready ISR to next-trigger call
(turnaround). These include preemption; ready includes the config transaction,
conversion, and interrupt latency. No timing text is emitted during acquisition.
Small timing instrumentation overhead is present and must be measured.

#ACQ reports each channel's count and first/last absolute 32-bit DRDY timestamps;
the harness calculates device acquisition rate independently of UDP delivery.
That span wraps after about 71 minutes, so keep diagnostic runs shorter than
one wrap. #ADC_EVENTS exposes event-queue overflow and stale pending events.
The harness stores these additions in summary.json; legacy firmware remains
supported. Diagnostics are suppressed if UART is still quiet at stop.
