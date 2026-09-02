"""Small COM9 capture tool for the EMG8 bracelet.

Examples:
    python serial_capture.py
    python serial_capture.py --seconds 45 --command 1
    python serial_capture.py --command 0 --seconds 3
"""

from __future__ import annotations

import argparse
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture EMG8 UART output")
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument(
        "--command",
        default="?",
        help="Command to send after opening (default: ?, use '' to send nothing)",
    )
    parser.add_argument(
        "--metadata-only",
        action="store_true",
        help="Print only #/H lines; suppress high-rate D data lines",
    )
    parser.add_argument(
        "--stop-after",
        type=float,
        default=None,
        help="Send command 0 after this many seconds (use with --command 1)",
    )
    args = parser.parse_args()

    try:
        port = serial.Serial(
            port=None,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1.0,
            dsrdtr=False,
            rtscts=False,
        )
        # Set modem-control outputs before assigning/opening the port. On
        # ESP32 USB-UART bridges, a DTR/RTS transition during open can assert
        # EN/GPIO0 and look like a firmware reboot.
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        try:
            # Do not discard pending output: it may contain the reset trace.
            if args.command:
                port.write(args.command.encode("ascii"))
                port.flush()

            started = time.monotonic()
            deadline = started + args.seconds
            stop_sent = False
            line_buffer = ""
            while time.monotonic() < deadline:
                if (
                    args.stop_after is not None
                    and not stop_sent
                    and time.monotonic() - started >= args.stop_after
                ):
                    port.write(b"0")
                    port.flush()
                    stop_sent = True
                data = port.read(port.in_waiting or 1)
                if data:
                    stamp = time.strftime("%H:%M:%S")
                    # Keep malformed/boot-rate bytes visible without making
                    # Windows' cp1252 console reject the capture.
                    text = data.decode("utf-8", errors="backslashreplace")
                    if args.metadata_only:
                        line_buffer += text
                        while "\n" in line_buffer:
                            line, line_buffer = line_buffer.split("\n", 1)
                            line = line.rstrip("\r")
                            if line.startswith("#") or line.startswith("H"):
                                sys.stdout.write(f"[{stamp}] {line}\n")
                    else:
                        sys.stdout.write(f"[{stamp}] {text}")
                    sys.stdout.flush()

            if args.metadata_only and line_buffer and (
                line_buffer.startswith("#") or line_buffer.startswith("H")
            ):
                print(line_buffer)
        finally:
            port.close()

    except serial.SerialException as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
