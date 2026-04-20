#!/usr/bin/env python3
"""Read serial output from SafeStep device and save to CSV."""

import serial
import serial.tools.list_ports
import csv
import re
import time
import sys
from datetime import datetime
from pathlib import Path

BAUD_RATE = 115200
# Matches: LiDAR:12.3 | F:45.6 L:78.9 R:12.3
PATTERN = re.compile(
    r"LiDAR:([\d.]+|OUT)\s*\|\s*F:([-\d.]+)\s+L:([-\d.]+)\s+R:([-\d.]+)"
)


def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(k in p.description.lower() for k in ("usb", "uart", "ch340", "cp210", "ftdi")):
            return p.device
    if ports:
        return ports[0].device
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found. Usage: python serial_to_csv.py [PORT]")
        sys.exit(1)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = Path(__file__).parent / "data"
    save_dir.mkdir(exist_ok=True)
    csv_path = save_dir / f"sensor_data_{timestamp}.csv"

    print(f"Connecting to {port} at {BAUD_RATE} baud...")
    print(f"Saving to {csv_path}")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(port, BAUD_RATE, timeout=2) as ser, \
         open(csv_path, "w", newline="") as f:

        writer = csv.writer(f)
        writer.writerow(["timestamp_s", "lidar_cm", "front_cm", "left_cm", "right_cm"])

        start = time.time()
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break

            if not line:
                continue

            print(line)
            m = PATTERN.search(line)
            if m:
                lidar_raw, f_val, l_val, r_val = m.groups()
                lidar_cm = lidar_raw if lidar_raw == "OUT" else float(lidar_raw)
                row = [
                    round(time.time() - start, 3),
                    lidar_cm,
                    float(f_val),
                    float(l_val),
                    float(r_val),
                ]
                writer.writerow(row)
                f.flush()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
