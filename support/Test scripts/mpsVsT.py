#!/usr/bin python3
# runs for a few sec and prints the time, mps to enc, mps input, Vbat to plot 

from pdb import run
import serial
import json
import time

stamp = 0
enc = 0
encLast = 0
diffEnc = 0
deltaEnc = 0
stampLast = 0
timeStart = 0
encStart = 0
vbat = 12.0
minVbat = 11.0
mpsIn = 0
runTime = 0

coeffA = 0.2
coeffB = 0.04

loopData = [ # runtime, M/S
    (8.0,0.4), 
    # (5000,0.2), 
    # (3333,0.3), 
    # (2500,0.4), 
    # (2000,0.5), 
    # (1666,0.6), 
    # (1428,0.7), 
    # (1250,0.8), 
    # (1111,0.9), 
    # (1000,1.0)
    ]

#######################################################
# Functions
#######################################################
def Stop() :
    cmd = json.dumps({"wd":1, "cv":[0,0]})
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush()

def SendSerial(cmd) :
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush() 


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

def printData():
    global encLast, stampLast, vbat, timeStart, mpsIn
    data = GetJsonData()
    #print(f"{data=}")
    if 'vbat' in data:
        vbat = data['vbat']

    if 'stamp' in data:
        stamp = data['stamp']
        enc = data['enc']
        mpsEsc = data['linx']
        pidInt = data['int']

        diffEnc = enc - encLast 
        encLast = enc

        diffStamp = stamp - stampLast
        stampLast = stamp
        timeSec = diffStamp/1000.0

        if timeSec>0 and abs(diffEnc<15000) :
            mpsEnc = (diffEnc/6000)/timeSec
            t = time.monotonic() - timeStart
            print(f"{t:.3f}, {mpsIn:.3f}, {mpsEsc:.3f}, {mpsEnc:.3f}, {pidInt:.4f}, {vbat:.2f}", flush=True)

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
# Set a 10sec command timeout
cmd = json.dumps({"mode":"cv", "wd":1})
SendSerial(cmd)
cmd = json.dumps({"wd":10000})
SendSerial(cmd)

# configure PID loop coefficients
cmd = json.dumps({"pid":[coeffA,coeffB]})
SendSerial(cmd)

# print data header
print("Tsec, mpsIn, mpsEsc, mpsEnc, pidInt, Vbat", flush=True)

timeStart = time.monotonic()

try:
    # Main loop to collect encoder counts vs battery voltage
    while vbat > minVbat :
        for runTime, mpsIn in loopData:
            # purge serial RX data
            while ser.in_waiting:
                ser.readline()

            # Start motor
            #print("Start motor")
            cmd = json.dumps({"cv":[mpsIn,0]})
            SendSerial(cmd)

            timeStop = time.monotonic()  + runTime
            while time.monotonic() < timeStop :
                printData()

            # Stop motor
            #print("Stop motor")
            mpsIn = 0
            cmd = json.dumps({"cv":[mpsIn,0]})
            SendSerial(cmd)

            timeStop = time.monotonic()  + runTime
            while time.monotonic() < timeStop :
                printData()


except KeyboardInterrupt:
    print("\nCtrl-C detected, stopping engine...")
    Stop()
finally:
    # Stop the engine if not already stopped and close serial
    Stop()
    ser.close()
