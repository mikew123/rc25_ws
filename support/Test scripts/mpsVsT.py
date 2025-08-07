#!/usr/bin python3
# runs for a few sec and prints the time, mps to enc, mps input, Vbat to plot 

from pdb import run
import serial
import json
import time

from sympy import true

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
cycleMps = 0
cycleStartT: float = 0

#PID coefficients
coeffA = 0.2
coeffB = 0.04
# Diff coefficients
coeffDA = 0.15
coeffDB = 0.075

wdTms = 10000 # watchdog time out
loopData = [ # runtime, M/S
    (5,0.1), 
    (5,0.2), 
    (5,0.3), 
    (5,0.4), 
    (5,0.5), 
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
    # print(f"Stop motor 0")
    cmd = json.dumps({"cv":[0,0]})
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush()

def SendSerial(cmd) :
    print(cmd)
    ser.write(cmd.encode('utf-8') + b'\n')
    ser.flush() 

def purgeSerial() :
    # purge serial RX data
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

def printData():
    global encLast, stampLast, vbat, timeStart, mpsIn
    global cycleStartT,cycleMps
    data = GetJsonData()
    #print(f"{data=}")
    if 'vbat' in data:
        vbat = data['vbat']

    if 'odom' in data:
        odom = data['odom']
        stamp = odom['stamp']
        enc = odom['enc']
        linX = odom['linx']
        angZ = odom['angz']

        diffEnc = enc - encLast 
        encLast = enc

        diffStamp = stamp - stampLast
        stampLast = stamp
        timeSec = diffStamp/1000.0

        cycleT = time.monotonic() - cycleStartT
        
        if timeSec>0 and abs(diffEnc<15000) :
            mpsEnc = (diffEnc/6000)/timeSec
            t = time.monotonic() - timeStart

            print(f"{t:.3f}, {cycleT:.3f}, {cycleMps}, {mpsIn:.3f}, {mpsEnc:.3f}, {linX:.3f}, {angZ:.4f}, {vbat:.2f}", flush=True)

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
cmd = json.dumps({"mode":"cv", "pid":[coeffA,coeffB,coeffDA,coeffDB], "wd":wdTms})
#cmd = json.dumps({"mode":"cv", "pid":[coeffA,coeffB,coeffDA,coeffDB]})
SendSerial(cmd)

time.sleep(5)
purgeSerial()

# print data header
print("Tsec, Tcyc, mpsCyc, mpsIn, mpsEnc, linX, angZ, Vbat", flush=True)

timeStart = time.monotonic()

try:
    # Main loop to collect encoder counts vs battery voltage
    while vbat > minVbat :
        for runTime, mpsIn in loopData:
            purgeSerial()

            cycleMps = mpsIn # save for plotting 
            # Start motor
            # print(f"Start motor, {mpsIn}")

            cmd = json.dumps({"cv":[mpsIn,0]})
            SendSerial(cmd)

            cycleStartT = time.monotonic()
            timeStop = time.monotonic()  + runTime
            while time.monotonic() < timeStop :
                printData() # from json message

            # Stop motor
            #print("Stop motor")
            mpsIn = 0
            cmd = json.dumps({"cv":[mpsIn,0]})
            SendSerial(cmd)

            timeStop = time.monotonic()  + runTime
            while time.monotonic() < timeStop :
                printData() # from json message


except KeyboardInterrupt:
    print("\nCtrl-C detected, stopping engine...")
    Stop()
finally:
    # Stop the engine if not already stopped and close serial
    Stop()
    ser.close()
