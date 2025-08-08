#!/usr/bin python3
# This script runs 1 meter ffwd and reverse and shows delta odom 

from pdb import run
import serial
import json
import time

import sys
import select

def GetJsonData():
    while True:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                #print(f"Received: {line=}")
                try:
                    data = json.loads(line)
                    break
                except json.JSONDecodeError:
                    continue
            time.sleep(0.05)
        data = json.loads(line)

        if 'millis' in data:
            return data


try:
	ser = serial.Serial("/dev/ttyACM0", 1000000)
except serial.SerialException as e:
	print(f"Error opening serial port: {e}")
	exit(1)

# Set engine controller to term mode to allow computer control
cmd = json.dumps({"mode": "term"})
ser.write(cmd.encode('utf-8') + b'\n')
ser.flush()

ocurrLast = 0
odiff = 0
ocurrStart = 0
millisStart = 0
runActive = False
runActiveLast = False
fwd = True

while True:
    print("Press 'f' for forward, 'r' for reverse, 'q' to quit")
    key = ''
    while key not in ['f', 'r', 'q']:
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1).strip()
    if key == 'q':
        break
    elif key == 'f':
        fwd = True
    elif key == 'r':
        fwd = False

    # purge serial RX data
    while ser.in_waiting:
        ser.readline()

    if fwd:
        cmd = json.dumps({"wd":10000, "cv":[0.1,0]});
    else:
        cmd = json.dumps({"wd":10000, "cv":[-0.1,0]});
    
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush()

    runActive = True

    while runActive:
        data = GetJsonData()
        #print(f"{data=}")

        ocurr = data['ocurr']
        odiff = ocurr - ocurrLast 
        ocurrLast = ocurr
        millis = data['millis']
        #print(f"{data=} {millis=} {millisStart=} {odiff=} {runActive=} {fwd= }")

        if runActiveLast == False:
            ocurrStart = ocurr
            millisStart = millis

        if(millis - millisStart) > 10000: # 10 seconds
            if odiff == 0:
                runActive = False
                odelta = ocurr - ocurrStart
                print(f"{fwd=} {ocurrStart=} {ocurr=} {odelta=}")

        runActiveLast = runActive

# Reset the throttle to 0 after testing
cmd = json.dumps({"thr": 0})
ser.write(cmd.encode('utf-8') + b'\n')
ser.flush()

ser.close()
