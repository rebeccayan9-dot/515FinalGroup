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
DURATION_S = 180  # 3 minutes
PATTERN = re.compile(
    r"LiDAR:([\d.]+|OUT)\s*\|\s*F:([-\d.]+)\s+L:([-\d.]+)\s+R:([-\d.]+)"
    r"\s*\|\s*AX:([-\d.]+)\s+AY:([-\d.]+)\s+AZ:([-\d.]+)"
    r"\s+GX:([-\d.]+)\s+GY:([-\d.]+)\s+GZ:([-\d.]+)"
)
BEHAVIORS = ["walking", "left_turn", "right_turn", "stop", "step_up", "step_down", "obstacle_avoid"]


def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(k in p.description.lower() for k in ("usb", "uart", "ch340", "cp210", "ftdi")):
            return p.device
    if ports:
        return ports[0].device
    return None


def ask_label():
    print("Select behavior label:")
    for i, b in enumerate(BEHAVIORS):
        print(f"  {i+1}. {b}")
    print("  0. Custom (type your own)")
    choice = input("Enter number or custom name: ").strip()
    if choice.isdigit():
        idx = int(choice)
        if idx == 0:
            return input("Custom label: ").strip().replace(" ", "_")
        if 1 <= idx <= len(BEHAVIORS):
            return BEHAVIORS[idx - 1]
    return choice.replace(" ", "_") or "unlabeled"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found. Usage: python serial_to_csv.py [PORT]")
        sys.exit(1)

    label = ask_label()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = Path(__file__).parent / "data"
    save_dir.mkdir(exist_ok=True)
    csv_path = save_dir / f"{label}_{timestamp}.csv"

    print(f"\nConnecting to {port} at {BAUD_RATE} baud...")
    print(f"Label: {label}")
    print(f"Saving to {csv_path}")
    print(f"Recording for {DURATION_S // 60} minutes. Press Ctrl+C to stop early.\n")

    with serial.Serial(port, BAUD_RATE, timeout=2) as ser, \
         open(csv_path, "w", newline="") as f:

        writer = csv.writer(f)
        writer.writerow(["timestamp_s", "lidar_cm", "front_cm", "left_cm", "right_cm",
                         "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "label"])

        start = time.time()
        while True:
            elapsed = time.time() - start
            if elapsed >= DURATION_S:
                print(f"\n3 minutes reached. Stopping.")
                break
            remaining = DURATION_S - elapsed
            print(f"\r  {int(remaining)}s remaining...", end="", flush=True)
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break

            if not line:
                continue

            print(f"\r{line:<60}")
            m = PATTERN.search(line)
            if m:
                lidar_raw, f_val, l_val, r_val, ax, ay, az, gx, gy, gz = m.groups()
                lidar_cm = lidar_raw if lidar_raw == "OUT" else float(lidar_raw)
                row = [
                    round(time.time() - start, 3),
                    lidar_cm,
                    float(f_val), float(l_val), float(r_val),
                    float(ax), float(ay), float(az),
                    float(gx), float(gy), float(gz),
                    label,
                ]
                writer.writerow(row)
                f.flush()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
