# test serial interface recovery after watchdog timeout
# tested by pulling a I2C wire on the tof-ctrl breadboard

import time
import serial

serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_45533065790A3B5A-if00"
baudrate = 1000000

# Open serial port to tof sensors controller over USB
serialOpen = False
while not serialOpen :
    try:
        ser = serial.Serial(serial_port, baudrate, timeout=1)
        print(f"Serial port {serial_port} opened.")
        serialOpen = True

    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        print("Try opening serial port again")
        
while 1 :
    # Check if a line has been received on the serial port
    err=False
    try :
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print(f"Received engine json: {received_data}")
    except Exception as ex:
        print(f"TOF sensors serial read failure : {ex}")
        err=True
    if err :
        try :
            ser.close()    
            ser = serial.Serial(serial_port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")


