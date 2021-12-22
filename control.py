import glob
import time
import RPi.GPIO as GPIO
import logging
import random

import paho.mqtt.client as mqtt_client

# configuration
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

sensors = ['28-0000065be5ef',
        '28-0000065c181d',
        '28-0000065cd66d']
device_files = {s_i:(base_dir + s_i + '/w1_slave') for s_i in sensors}

pin_compSwitch = 26 # GPIO Output - switching the frige compressor on
pin_doorSensor = 19 # GPIO Input - determine if door is open
coolingOn_temp = 80
coolingOff_temp = 75
compOn_minMinutes = 0.2 #2
compOn_maxMinutes = 1 #15
compOff_minMinutes = 0.5 #10

# mqtt config
broker = '192.168.68.109'
port = 1883
topic = 'kegerator0/sensor_float'
client_id = 'python-mqtt-1'

# initialize mqtt message json schema
msgSensor_format = """{{
    "sensors": {{
        "intTemp": {_intTemp},
        "extTemp": {_extTemp},
        "compTemp": {_compTemp},
        "compAmps": {_compAmps},
        "kegWeight": {_kegWeight} }},
    "tags": {{
        "compState": "{_compState}",
        "doorState": "{_doorState}"}},
    "ts": {_ts} }}"""
msgEvent_format = """{{
    "event_type": "{_type}",
    "event_current":"{_current}",
    "event_last": "{_last}",
    "lastDuration": {_duration},
    "ts": {_ts} }}"""
msgConfig_format = """{{
    "compOffTemp": {_compOffTemp},
    "compOnTemp": {_compOnTemp},
    "compOnMinTime": {_compOnMinTime},
    "compOnMaxTime": {_compOnMaxTime},
    "compOffMinTime": {_compOffMinTime},
    "ts": {_ts} }}"""

#TODO
## inputs
### inside_temp
### outside_temp
### comp_current
### comp_temp
### door_open
### keg_mass
## outputs
### compOn
## configuration
### coolingOn_temp - temp to start cooling
### coolingOff_temp - temp to stop cooling
### compOn_minMinutes - min time for compressor to be on
### compOn_maxMinutes - max time for compressor to be on
### compOff_minMinutes - min time for compressor to be off
## code
### thermostat -- DONE
### add logging to file -- TODO add rollover, limit file size
#### initialization
#### better formatting of log file entry -- DONE
### add pushing data to MQTT endpoint
### using config file for configuration parameters instead of in this script

def initialize():
    try:
        # set up logging
        logging.basicConfig(filename="control.log",
                format='%(asctime)s %(levelname)-8s %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S',
                level=logging.DEBUG)
        # set up GPIO
        logging.debug("before GPIO init")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_compSwitch, GPIO.OUT)
        GPIO.setup(pin_doorSensor, GPIO.IN)
        logging.info("init success")
        # TODO - insert log entry for configuration
        logging.info("initialization parameters\ncoolingOn_temp : {}\ncoolingOff_temp : {}\ncompOn_minMinutes : {}\ncompOn_maxMinutes : {}\ncompOff_minMinutes : {}".format(coolingOn_temp,coolingOff_temp,compOn_minMinutes,compOn_maxMinutes,compOff_minMinutes))
        print("ok")
    except:
        logging.error("ERROR in init")
        print("bad")

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker")
            logging.debug("Connected to MQTT Broker")
        else:
            print("Failed to connect, return code %d\n", rc)
            logging.debug("Failed to connect, return code %d\n", rc)
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def comp_switch(compOn):
    if compOn :
        GPIO.output(pin_compSwitch,GPIO.HIGH)
    else :
        GPIO.output(pin_compSwitch,GPIO.LOW)

def read_temp_raw(device_file):
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp(device_file):
    lines = read_temp_raw(device_file)
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c, temp_f

def thermostat(T, current_stateOn, dt_state):
    # T = inside temp of fridge in F
    # calculate duration of state for dt_state, time in state
    dt_state[2] = (time.time() - dt_state[1])/60 # minutes , dt_state = [current state, start time, duration in state]
    if T>coolingOn_temp and current_stateOn==False and dt_state[2]>compOff_minMinutes:
        # normal compressor to ON
        next_stateOn = True
        logging.debug("1 - cur_state, next_state, dur_state : {} --> {} - {} mins".format(current_stateOn,next_stateOn,dt_state[2]))
        dt_state = [next_stateOn, time.time(), 0]
    elif T<coolingOff_temp and current_stateOn==True and dt_state[2] > compOn_minMinutes:
        # normal compressor to OFF
        next_stateOn = False
        logging.debug("2 - cur_state, next_state, dur_state : {} --> {} - {} mins".format(current_stateOn,next_stateOn,dt_state[2]))
        dt_state = [next_stateOn, time.time(), 0]
    elif dt_state[0]==True and dt_state[2] > compOn_maxMinutes:
        # timeout compressor to OFF
        next_stateOn = False
        logging.debug("3 - cur_state, next_state, dur_state : {} --> {} - {} mins".format(current_stateOn,next_stateOn,dt_state[2]))
        dt_state = [next_stateOn, time.time(), 0]
    else:
        # no state change
        next_stateOn = current_stateOn
    
    return next_stateOn, dt_state


def log_data(intTemp, extTemp, compTemp, compAmps, kegWeight, compState, doorState, ts):
    logging.debug(f"""ts -- intTemp, extTemp, compTemp, compAmps, kegWeight, compState, doorState : ({ts} --
            {intTemp}, {extTemp}, {compTemp}, {compAmps}, {kegWeight}, {compState}, {doorState})""")


#def mqtt_pub(client):
    # publish sensor, event, and config data to mqtt broker

def main():
    # set inital last state of compressor to OFF
    current_stateOn = False
    dt_state = [False,0,0] # (state,start_time,duration), initialize tracker for time in state
    
    # mqtt setup
    client = connect_mqtt()
    client.loop_start()
    logging.info("connection to mqtt broker success")

    # runtime code
    while True:
        # read temperature from sensor
        _,Tf = read_temp(device_files['28-0000065be5ef'])
        _,Tf_ext = read_temp(device_files['28-0000065c181d'])
        _,Tf_comp = read_temp(device_files['28-0000065cd66d'])
        #logging.debug("temp is {}".format(Tf))
        #Tf = float(input("Enter Tf : "))
        
        # determine compressor state
        compOn, dt_state = thermostat(Tf,current_stateOn, dt_state)
        
        #TODO - move to a method
        # log
        logging.debug("TempF, next_state, state, state_ts, state_duration is ({}, {}, {}, {}, {})".format(Tf,compOn,current_stateOn,round(dt_state[1],1),round(dt_state[2],1)))
        
        # change compressor state
        comp_switch(compOn)
        # save current cycle's compressor state for next cycle
        current_stateOn = compOn
        
        #TODO - move to a method
        # publish to mqtt
        result = client.publish(topic, Tf)
        status = result[0]
        if status == 0:
            logging.debug(f"Send Temp {Tf} to '{topic}'")
        else:
            logging.debug(f"Failed to send message to topic '{topic}'")


        time.sleep(1)

if __name__ == "__main__":
    ## initialization
    init_error = initialize()
    # main program
    main()
