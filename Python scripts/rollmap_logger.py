"""
rollmap_logger.py

Reads Serial output from the ESP8266 (MPU6050 + AS5600 sketch) and logs
every "DATA,<millis>,<abs_rotations>,<roll>" line into a local CSV file.

Setup:
    pip install pyserial

Usage:
    1. Edit SERIAL_PORT below to match your board.
       - Windows: something like "COM3", "COM5", etc.
         (Check Device Manager -> Ports (COM & LPT))
       - macOS:   something like "/dev/tty.usbserial-1410" or "/dev/tty.SLAB_USBtoUART"
         (Run `ls /dev/tty.*` in Terminal while the board is plugged in)
       - Linux:   something like "/dev/ttyUSB0" or "/dev/ttyACM0"
         (Run `ls /dev/tty*` in a terminal while the board is plugged in)
    2. Make sure the Arduino Serial Monitor is CLOSED - only one program
       can hold the serial port open at a time.
    3. Run:  python rollmap_logger.py
    4. Press Ctrl+C to stop. The CSV is saved incrementally as data comes
       in, so it's safe to stop at any point.
"""

import csv
import sys
import time

import serial

# ---- Settings you may need to change ----
SERIAL_PORT = "COM8"          # <-- change this to match your board's port
BAUD_RATE = 115200               # must match Serial.begin(115200) in the sketch
OUTPUT_CSV = "rollmap_live.csv"
DATA_PREFIX = "DATA,"
# -------------------------------------------


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open serial port '{SERIAL_PORT}': {e}")
        print("Check that the port name is correct and the Arduino Serial "
              "Monitor is closed.")
        sys.exit(1)

    # Give the board a moment after the port opens (it often auto-resets)
    time.sleep(2)

    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    print(f"Logging DATA rows to '{OUTPUT_CSV}'. Press Ctrl+C to stop.\n")

    row_count = 0

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["millis", "abs_rotations", "roll"])
        f.flush()

        try:
            while True:
                raw_line = ser.readline()
                if not raw_line:
                    continue  # timeout with no data - just keep waiting

                line = raw_line.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # Echo everything so you can still see status/motor messages
                print(line)

                if line.startswith(DATA_PREFIX):
                    fields = line[len(DATA_PREFIX):].split(",")
                    if len(fields) == 3:
                        writer.writerow(fields)
                        f.flush()
                        row_count += 1

        except KeyboardInterrupt:
            print(f"\nStopped. {row_count} rows saved to '{OUTPUT_CSV}'.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
