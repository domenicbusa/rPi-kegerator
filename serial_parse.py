#!/usr/bin/env python3
import serial
import json

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print("raw: " + line)
            try:
                msg = json.loads(line)
                print(f"{msg[0]['name']} - Pavg: {msg[0]['vals']['Irms']}")
            except:
                print("error in json read")
