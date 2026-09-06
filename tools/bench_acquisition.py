"""SD-free firmware benchmark. Uses COM9 for control and UDP for full records.

Never resets the board or enables SD. Refuses to start if SD is mounted.
Outputs metadata.json, serial.jsonl, udp.bin and summary.json in a new directory.
UDP capture framing is host monotonic_ns:u64, length:u16, then datagram bytes.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import json
from pathlib import Path
import socket
import struct
import subprocess
import time

RAW = ((0, 3), (1, 3), (0, 3), (1, 3))
ENV = ((1, 2), (0, 2), (1, 2), (0, 2))
HEADER = struct.Struct('<2sBBIHH')
SAMPLE = struct.Struct('<IBBh')


class Records:
    def __init__(self):
        self.timestamps = collections.defaultdict(list)
        self.packets = collections.Counter()
        self.gaps = collections.Counter()
        self.reordered = collections.Counter()
        self.duplicates = collections.Counter()
        self.seen = collections.defaultdict(set)
        self.base_seq = {}
        self.last_seq = {}
        self.invalid = 0

    def feed(self, pkt):
        if len(pkt) < HEADER.size:
            self.invalid += 1
            return
        magic, version, kind, seq, count, _ = HEADER.unpack_from(pkt)
        size = 20 if kind == 2 else 8
        if magic != b'E8' or version != 1 or kind > 2 or not count or len(pkt) != 12 + count * size:
            self.invalid += 1
            return
        base = self.base_seq.setdefault(kind, seq)
        relative = (seq - base) & 0xffffffff
        if seq in self.seen[kind]:
            self.duplicates[kind] += 1
            return
        if relative > 0x80000000:
            self.reordered[kind] += 1
            return  # older than the initial subscription boundary
        if kind in self.last_seq and relative < self.last_seq[kind]:
            self.reordered[kind] += 1
        self.seen[kind].add(seq)
        self.last_seq[kind] = max(relative, self.last_seq.get(kind, 0))
        self.gaps[kind] = self.last_seq[kind] + 1 - len(self.seen[kind])
        self.packets[kind] += 1
        for offset in range(12, len(pkt), size):
            ts = struct.unpack_from('<I', pkt, offset)[0]
            if kind < 2:
                _, adc, ch, _ = SAMPLE.unpack_from(pkt, offset)
                if adc >= 4 or ch >= 4 or ch not in (RAW if kind == 0 else ENV)[adc]:
                    self.invalid += 1
                    continue
                key = f'{kind}:{adc}:{ch}'
            else:
                key = '2:imu'
            self.timestamps[key].append(ts)

    def summary(self):
        channels = {}
        for key, ts in sorted(self.timestamps.items()):
            # Sort by device time so network reordering cannot inflate the rate.
            ordered = sorted((t - ts[0]) & 0xffffffff for t in ts)
            delta = [b - a for a, b in zip(ordered, ordered[1:])]
            valid = sorted(d for d in delta if 0 < d < 0x80000000)
            duration = ordered[-1] - ordered[0]
            unwrapped = [ts[0] + t for t in ordered]
            bins = collections.Counter(t // 10000000 for t in unwrapped)
            complete = [bins[b] / 10 for b in range((unwrapped[0] + 9999999) // 10000000,
                                                  unwrapped[-1] // 10000000)]
            channels[key] = {
                'received': len(ts), 'first_us': ts[0], 'last_us': unwrapped[-1],
                'received_hz': (len(ts) - 1) * 1e6 / duration if duration else None,
                'nonpositive_or_backward': len(delta) - len(valid),
                'interval_us': {str(p): valid[min(len(valid)-1, int((len(valid)-1)*p/100))]
                                for p in (50, 95, 99, 100)} if valid else {},
                'complete_10s_received_hz': complete,
            }
        return {'channels': channels, 'packets': dict(self.packets),
                'packet_gaps': dict(self.gaps), 'reordered_packets': dict(self.reordered),
                'duplicate_packets': dict(self.duplicates),
                'invalid_records_or_packets': self.invalid}



def firmware_diagnostics(lines):
    timing, acquired, events = {}, {}, {}
    for line in lines:
        if line.startswith('#TIMING:'):
            fields = line.split(':', 1)[1].split(',')
            adc, name = fields[:2]
            count, total, minimum, maximum, *bins = map(int, fields[2:])
            timing[adc + ':' + name] = dict(count=count, total_us=total,
                mean_us=total / count if count else None, min_us=minimum,
                max_us=maximum, bins=bins)
        elif line.startswith('#ACQ:'):
            adc, ch, count, first, last = map(int, line.split(':', 1)[1].split(','))
            span = (last - first) & 0xffffffff
            acquired[f'{adc-1}:{ch}'] = dict(count=count, first_us=first, last_us=last,
                hz=(count - 1) * 1e6 / span if count > 1 and span else None)
        elif line.startswith('#ADC_EVENTS:'):
            adc, dropped, spurious = map(int, line.split(':', 1)[1].split(','))
            events[str(adc)] = dict(queue_drops=dropped, spurious=spurious)
    return dict(timing=timing, device_acquisition=acquired, adc_events=events)


def run(args):
    import serial
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    metadata = dict(vars(args), utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    raw_channels=RAW, envelope_channels=ENV,
                    rate_note='CNT uses host REC-to-stop-send time; UDP uses device timestamps')
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    ser = serial.Serial(port=None, baudrate=460800, timeout=0, write_timeout=2,
                        dsrdtr=False, rtscts=False)
    ser.dtr = False
    ser.rts = False
    ser.port = args.port
    sock = None
    logs = []
    buf = bytearray()
    records = Records()
    started = False
    status = None
    start_time = None
    stop_time = None
    counters = {}
    last_sub = 0.0
    failure = None
    serial_file = (output / 'serial.jsonl').open('w', encoding='utf-8')
    udp_file = (output / 'udp.bin').open('wb')

    def send(cmd):
        ser.write(cmd.encode('ascii'))
        serial_file.write(json.dumps({'host_ns': time.perf_counter_ns(), 'tx': cmd}) + '\n')
        serial_file.flush()

    def poll():
        nonlocal status, start_time, last_sub
        buf.extend(ser.read(ser.in_waiting or 1))
        while b'\n' in buf:
            line, _, rest = buf.partition(b'\n')
            buf[:] = rest
            line = line.decode('ascii', errors='backslashreplace').strip()
            now = time.perf_counter()
            if not line.startswith('D,'):
                logs.append(line)
                serial_file.write(json.dumps({'host_ns': time.perf_counter_ns(), 'rx': line}) + '\n')
                serial_file.flush()
            if line.startswith('#STATUS:'):
                status = [int(v) for v in line.split(':', 1)[1].split(',')]
            if line == '#REC':
                start_time = now
            if line.startswith('#CNT:'):
                values = [int(v) for v in line.split(':', 1)[1].split(',')]
                counters[values[0] - 1] = values[1:]
            if 'Guru Meditation' in line or line.startswith('#BOOT:'):
                raise RuntimeError('Unexpected device reset: ' + line)
        if sock:
            if time.perf_counter() - last_sub > 2:
                sock.sendto(b'HI', (args.host, 3333))
                last_sub = time.perf_counter()
            for _ in range(500):
                try:
                    pkt = sock.recv(2048)
                except BlockingIOError:
                    break
                udp_file.write(struct.pack('<QH', time.perf_counter_ns(), len(pkt)) + pkt)
                if started:
                    records.feed(pkt)

    def wait(seconds):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            poll()
            time.sleep(.001)

    try:
        ser.open()
        send('U1')
        send('?')
        wait(1)
        if status is None or status[1] != 0 or status[2] != 0:
            raise RuntimeError(f'Requires idle device with SD unavailable; status={status}')
        # Reset networking for an actual no-subscriber condition and empty batches.
        send('W0')
        wait(1)
        if args.condition != 'off':
            send('W1')
            wait(2)
        if args.condition in ('udp', 'quiet'):
            if args.wifi_profile:
                subprocess.run(['netsh', 'wlan', 'connect', 'name=' + args.wifi_profile],
                               check=True, timeout=10, capture_output=True)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            sock.bind(('', 0))
            sock.setblocking(False)
            # Allow Windows to reconnect to the saved SoftAP after W0/W1.
            wait(args.connect_wait)
            if not any(line.startswith('#NET:') and 'TX=' not in line for line in logs):
                raise RuntimeError('No UDP subscription acknowledgement; join bracelet Wi-Fi')
        counters.clear()
        started = True
        send(str(args.mode))
        deadline = time.perf_counter() + 8
        while start_time is None and time.perf_counter() < deadline:
            wait(.05)
        if start_time is None:
            raise RuntimeError('No #REC acknowledgement')
        if args.condition == 'quiet':
            send('U0')
        print(f'RUNNING {args.condition} mode={args.mode} {args.seconds}s -> {output}', flush=True)
        wait(max(0, start_time + args.seconds - time.perf_counter()))
        if args.condition == 'quiet':
            send('U1')
            wait(.15)
        stop_time = time.perf_counter()
        send('0')
        wait(1.0)  # collect CNT plus all 30ms partial UDP batches
        started = False
        if len(counters) != 4:
            raise RuntimeError('Missing per-ADC stop counters')
        if args.condition in ('udp', 'quiet') and not records.packets:
            raise RuntimeError('No received UDP data')
    except Exception as exc:
        failure = str(exc)
    finally:
        if ser.is_open:
            try:
                send('U1')
                if started:
                    send('0')
                    wait(.5)
                send('W0')
                wait(.5)
                send('?')
                wait(.2)
            except Exception as exc:
                failure = failure or str(exc)
            ser.close()
        if sock:
            sock.close()
        serial_file.close()
        udp_file.close()
    result = records.summary()
    result.update(firmware_diagnostics(logs))
    elapsed = stop_time - start_time if stop_time is not None and start_time is not None else None
    result.update(condition=args.condition, mode=args.mode, host_window_s=elapsed,
                  failure=failure, final_status=status, counts=counters)
    result['host_timed_acquired_hz'] = {
        f'{adc}:{ch}': round(values[ch] / elapsed, 3)
        for adc, values in sorted(counters.items()) for ch in range(4)
        if elapsed and values[ch]
    }
    acquired = sum(sum(v[:4]) for v in counters.values())
    received = sum(len(v) for k, v in records.timestamps.items() if k != '2:imu')
    result['adc_delivery_fraction'] = received / acquired if acquired and args.condition in ('udp', 'quiet') else None
    result['i2c_errors'] = sum(v[4] for v in counters.values())
    result['retriggers'] = sum(v[5] for v in counters.values())
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('failure', 'host_timed_acquired_hz',
          'adc_delivery_fraction', 'packet_gaps', 'i2c_errors', 'retriggers')}), flush=True)
    return 1 if failure else 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--host', default='192.168.4.1')
    ap.add_argument('--condition', choices=('off', 'nosub', 'udp', 'quiet'), default='udp')
    ap.add_argument('--mode', type=int, choices=(1, 2, 3), default=1)
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--connect-wait', type=float, default=12)
    ap.add_argument('--wifi-profile', help='Reconnect this existing Windows Wi-Fi profile after W1')
    ap.add_argument('--output', required=True)
    raise SystemExit(run(ap.parse_args()))
