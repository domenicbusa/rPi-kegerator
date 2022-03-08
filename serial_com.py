#!/usr/bin/env python3
import serial
import json
import time

READ_INTERVAL = 10 # seconds between sensor readings
SERIAL_READ_TIMEOUT = 5 # seconds to wait for response
reqSerialMsg = {"ready": True, "last_ts": 0}


def read_sensors(ser): 
    # send request over serial
    ser.write("data\n".encode('utf-8'))

    # wait for serial response
    listen_startTime = time.time()
    while True :
        # check serial for response 
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            #print(f"raw: {line} \n")
            # decode message to json
            try:
                msg = json.loads(line)
            except:
                print("error in json decode")
            # determine message type
            if msg[0]["name"] == "REQUEST_RCVD":
                # skip current loop iteration, wait for data payload
                continue

            # decode data payload json
            try:
                sensor_dict = {'intTemp' : msg[1]["vals"]["tempF"],
                    'extTemp' : msg[2]["vals"]["tempF"],
                    'compTemp': msg[3]["vals"]["tempF"],
                    'compAmps': msg[0]["vals"]["Irms"],
                    'kegWeight': -1,
                    'doorState': "CLOSED",
                    'ts': time.time_ns()}
            except:
                print("error in json parse of serial response")
            # successful read
            break
        # exit loop if timeout elapses
        if time.time() - listen_startTime > SERIAL_READ_TIMEOUT:
            print("error in serial read, no response")
            break
    return sensor_dict

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    reqSerialMsg = {"ready": True, "last_ts": 0}
    
    while True:
        time_since_last = time.time()-reqSerialMsg["last_ts"] # seconds
        # check if request timeout has elapsed
        if time_since_last > READ_INTERVAL :
            sensor_dict = read_sensors(ser)
            # update timer dict
            reqSerialMsg["last_ts"] = time.time()
            print(sensor_dict)
            print(f"---- ts = {round(time.time(),0)} ----\n")
        #time.sleep(READ_INTERVAL)
        '''
        time_since_last = time.time()-reqSerialMsg["last_ts"] # seconds
        # check if request timeout has elapsed
        if time_since_last > READ_INTERVAL :
            # request data
            if reqSerialMsg["ready"] :
                ser.write("data\n".encode('utf-8'))
            # disable requesting till timeout
            reqSerialMsg["ready"] = False
            reqSerialMsg["last_ts"] = time.time()

        # check serial for response 
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(f"raw: {line} \n")
            
            # decode message to json
            try:
                msg = json.loads(line)
            except:
                print("error in json decode")
            
            # determine message type
            if msg[0]["name"] == "REQUEST_RCVD":
                # skip confirmation message
                continue
            
            # parse json data message
            try:
                print(f"{msg[0]['name']} - Pavg: {msg[0]['vals']['Irms']}")
            except:
                print("error in json parse")
            # enable requesting
            reqSerialMsg["ready"] = True
        '''

