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

# set throttle to 25% and delay for 10 rotations
cmd = json.dumps({"thr": 25})
ser.write(cmd.encode('utf-8') + b'\n')
ser.flush()

#time.sleep(848)

currT = time.time()
startT = currT
while currT < startT + 84.800 :
    currT = time.time()
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        print(f"{line}")

# Reset the throttle to 0 after testing
cmd = json.dumps({"thr": 0})
ser.write(cmd.encode('utf-8') + b'\n')
ser.flush()

ser.close()

