#!/usr/bin python3
# This script cycles the engine controller through various throttle values
# and prints out the Odom and RPM values to be analyzed in a spreadsheet

import serial
import json
import time

try:
	ser = serial.Serial("/dev/ttyACM0", 1000000)
except serial.SerialException as e:
	print(f"Error opening serial port: {e}")
	exit(1)

# Set engine controller to term mode to allow computer control
cmd = json.dumps({"mode": "term"})
ser.write(cmd.encode('utf-8') + b'\n')
ser.flush()

for throttle in range(20, 101, 1):
    cmd = json.dumps({"thr": throttle})
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush()

    for _ in range(10):
        # Wait until a valid JSON string is read from the serial port
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                try:
                    data = json.loads(line)
                    break
                except json.JSONDecodeError:
                    continue
            time.sleep(0.05)
        data = json.loads(line)
        print (f"Tms: {data['millis']}, THR: {throttle}, RPM: {data['rpm']}, Odom: {data['ocurr']}")

# Reset the throttle to 0 after testing
cmd = json.dumps({"thr": 0})
ser.write(cmd.encode('utf-8') + b'\n')
ser.flush()

ser.close()

