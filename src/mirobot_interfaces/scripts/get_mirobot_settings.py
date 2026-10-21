#!/usr/bin/env python3

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial", file=sys.stderr)
    print("Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def main() -> int:
    parser = argparse.ArgumentParser(description="Query Mirobot settings using $$")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--idle-timeout", type=float, default=1.0, help="Stop after this many seconds without new data")
    parser.add_argument("--startup-wait", type=float, default=2.0, help="Wait after opening serial port before sending $$")
    parser.add_argument("--no-toggle-status", action="store_true", help="Do not send O104=0/O104=1 around $$")
    parser.add_argument(
        "--commands",
        default="$$,$I,$N,$G,M503",
        help="Comma-separated commands to query (default: $$,$I,$N,$G,M503)",
    )
    args = parser.parse_args()

    commands = [c.strip() for c in args.commands.split(",") if c.strip()]
    if not commands:
        print("No commands configured. Use --commands.", file=sys.stderr)
        return 1

    def read_command_response(ser: serial.Serial, command_text: str) -> tuple[list[str], bool]:
        got_data = False
        idle_deadline = time.monotonic() + args.idle_timeout
        response_lines: list[str] = []

        while True:
            line = ser.readline()
            if line:
                got_data = True
                idle_deadline = time.monotonic() + args.idle_timeout
                text = line.decode("utf-8", errors="replace").rstrip()
                response_lines.append(text)
                print(text)
                continue

            if time.monotonic() >= idle_deadline:
                break

        return response_lines, got_data

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            time.sleep(args.startup_wait)
            ser.write(b"\r\n\r\n")
            ser.flush()
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            if not args.no_toggle_status:
                ser.write(b"O104=0\r\n")
                ser.flush()
                time.sleep(0.15)

            print(f"Connected to {args.port} @ {args.baud}")
            print("--- Parameter query ---")

            any_data = False
            parameter_lines: list[str] = []

            for command in commands:
                print(f">>> {command}")
                ser.write((command + "\r\n").encode("utf-8"))
                ser.flush()
                lines, got_data = read_command_response(ser, command)

                if not got_data:
                    print("(no response)")
                else:
                    any_data = True
                    for text in lines:
                        if "=" in text or text.startswith("$") or text.startswith("["):
                            parameter_lines.append(text)
                print("---")

            if not any_data:
                print("No response received. Check port, power, and whether another node is using the serial device.")

            if parameter_lines:
                print("--- Extracted parameter-like lines ---")
                for line in parameter_lines:
                    print(line)
            else:
                print("No parameter-style lines found in responses.")

            if not args.no_toggle_status:
                ser.write(b"O104=1\r\n")
                ser.flush()

            print("--- End ---")
            return 0
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())