"""Resume the SD-free 8 x 4 baseline matrix; never overwrite completed captures."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True)
    ap.add_argument('--wifi-profile', required=True)
    ap.add_argument('--repeat', type=int, default=8)
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--conditions', default='off,nosub,udp,quiet')
    args = ap.parse_args()
    conditions = args.conditions.split(',')
    if any(c not in ('off', 'nosub', 'udp', 'quiet') for c in conditions):
        ap.error('unknown condition')
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    for rep in range(1, args.repeat + 1):
        for condition in conditions:
            output = root / f'{condition}-{rep:02d}'
            summary = output / 'summary.json'
            if summary.exists() and json.loads(summary.read_text()).get('failure') is None:
                print(f'SKIP completed {output.name}', flush=True)
                continue
            # Preserve failed/interrupted captures and give retries their own names.
            attempt = 1
            while output.exists():
                output = root / f'{condition}-{rep:02d}-retry{attempt}'
                summary = output / 'summary.json'
                if summary.exists() and json.loads(summary.read_text()).get('failure') is None:
                    break
                attempt += 1
            else:
                command = [sys.executable, '-B', str(Path(__file__).with_name('bench_acquisition.py')),
                           '--condition', condition, '--seconds', str(args.seconds),
                           '--wifi-profile', args.wifi_profile, '--output', str(output)]
                print(f'START {output.name}', flush=True)
                result = subprocess.run(command)
                if result.returncode:
                    print('STOP: failed run saved; correct the cause before resuming.', flush=True)
                    return result.returncode
    print('MATRIX COMPLETE', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
