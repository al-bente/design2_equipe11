#!/usr/bin/env python3

import serial
import time


ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    timeout=1
)

while True:

    try:

        data = ser.read(128)
        print(data)

    except Exception as e:
        print("Something went wrong:", e)
        break


ser.close()
