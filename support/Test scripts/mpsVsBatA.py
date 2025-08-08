#!/usr/bin python3
# This script runs multiple M/S and runTimes to travel 1M (on bench)
# the voltage is recorded to understand how battery level affect M/S
# updated to run for 1/2sec before starting measuremment run time is 
# measured using the time stamps and a stop command is given

#from pdb import run
import serial
import json
import time

# import sys
# import select

stamp = 0
enc = 0
encLast = 0
diffEnc = 0
deltaEnc = 0
stampStart = 0
encStart = 0
runActive = False
runActiveLast = False
vbat = 12.0
minVbat = 11.0
mps = 0.1
runTime = 10000
loopData = [
    (10000,0.1), 
    (5000,0.2), 
    (3333,0.3), (2500,0.4), 
    (2000,0.5), 
    (1666,0.6), 
    (1428,0.7), 
    (1250,0.8), 
    (1111,0.9), 
    (1000,1.0)
    ]

#######################################################
# Functions
#######################################################

def SendSerial(cmd) :
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush() 

def SendJsonCmd(jsonCmd) :
    cmd = json.dumps(jsonCmd)
    SendSerial(cmd)

def PurgeSerial():
    while ser.in_waiting:
        ser.readline()

def GetJsonData():
    while True :
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            #print(f"Received: {line=}")
            try:
                data = json.loads(line)
                return data # exit loop if line is json formated
            except json.JSONDecodeError:
                continue
        time.sleep(0.01)

#######################################################
# Execution starts here
#######################################################

# Configure serial port
#SerialPortName = "/dev/ttyACM0"
SerialPortName = "/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6625887D37C3E30-if00"

try:
	ser = serial.Serial(SerialPortName, 1000000)
except serial.SerialException as e:
	print(f"Error opening serial port: {e}")
	exit(1)

# Set engine controller to term mode to allow computer control
cmd = json.dumps({"mode": "term"})
SendSerial(cmd)

# Send stop command - there seems to be an issue after Pico reset
cmd = json.dumps({"wd":1, "cv":[0,0]}) # STOP
SendSerial(cmd)

# print data header
print("TimeMs, RunTime, MPS, Vbat, DeltaEnc", flush=True)

try:
    # Main loop to collect encoder counts vs battery voltage
    while vbat > minVbat :
        for runTime, mps in loopData:
            PurgeSerial()

            rt = runTime+2
            jsonCmd = {"wd":rt, "cv":[mps,0]}
            SendJsonCmd(jsonCmd)

            time.sleep(0.5) # let mechanics settle
            PurgeSerial()

            # send command to cause battery voltage update during motor running
            jsonCmd = {"wd":rt, "cv":[mps,0]}
            SendJsonCmd(jsonCmd)

            runActive = True

            while runActive:
                data = GetJsonData()
                #print(f"{data=}")
                if 'vbat' in data:
                    vbat = data['vbat']

                if 'stamp' in data:
                    stamp = data['stamp']
                    enc = data['enc']
                    diffEnc = enc - encLast 
                    encLast = enc

                if runActiveLast == False:
                    stampStart = stamp
                    encStart = enc

                if(stamp - stampStart) > runTime :
                    jsonCmd = {"wd":1, "cv":[0,0]} # STOP
                    SendJsonCmd(jsonCmd)
                    if diffEnc == 0:
                        deltaEnc = enc - encStart
                        print(f"{stamp}, {runTime}, {mps}, {vbat:.2f}, {deltaEnc}", flush=True)
                        time.sleep(0.5)
                        runActive = False

                runActiveLast = runActive

except KeyboardInterrupt:
    print("\nCtrl-C detected, stopping engine...")
    cmd = json.dumps({"wd":1, "cv":[0,0]})
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush()

finally:
    # Stop the engine if not already stopped and close serial
    cmd = json.dumps({"wd":1, "cv":[0,0]})
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush()

    ser.close()
