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

configData = {'pin_compSwitch':pin_compSwitch,
        'pin_doorSensor':pin_doorSensor,
        'coolingOn_temp':coolingOn_temp,
        'coolingOff_temp':coolingOff_temp,
        'compOn_minMinutes':compOn_minMinutes,
        'compOn_maxMinutes':compOn_maxMinutes,
        'compOff_minMinutes':compOff_minMinutes,
        'ts':time.time_ns()} #TODO - add temp sensor configs, broker, port, topics

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

#def thermostat(T, current_stateOn, dt_state):
def thermostat(T, dt_state, client):
    # inputs
        # T - inside temp of fridge in F
        # current_stateOn - 
        # dt_state = [next_stateOn, current_timestamp, current_state_duration]
            # next_stateOn - state of compressor (ON/OFF) for next cycle
            # current_timestamp - 
            # current_state_duration - duration of current state
        # dt_state = {'state': current state of compressor True==ON,
        #            'start_ts': start time of current state,
        #            'duration': duration (minutes) of current state}}

    # update duration of state for dt_state, time in state
#    dt_state[2] = (time.time() - dt_state[1])/60 # minutes , dt_state = [current state, start time, duration in state]
    dt_state['duration'] = (time.time() - dt_state['start_ts'])/60 # minutes
    current_stateOn = dt_state['state'] # store state of compressor before thermostat state change logic
    current_stateDuration = dt_state['duration']
    next_stateOn = current_stateOn
    state_change = 0
    # state change logic
    if T>coolingOn_temp and current_stateOn==False and dt_state['duration']>compOff_minMinutes: #dt_state[2]>compOff_minMinutes:
        # normal compressor to ON
        next_stateOn = True
        state_change = 1
        #logging.debug("1 - cur_state, next_state, dur_state : {} --> {} - {} mins".format(current_stateOn,next_stateOn,dt_state['duration']))
        #dt_state = [next_stateOn, time.time(), 0]
        #dt_state.update({'state': next_stateOn,'start_ts': time.time(), 'duration': 0})
    elif T<coolingOff_temp and current_stateOn==True and dt_state['duration']>compOn_minMinutes: #dt_state[2] > compOn_minMinutes:
        # normal compressor to OFF
        next_stateOn = False
        state_change = 2
        #logging.debug("2 - cur_state, next_state, dur_state : {} --> {} - {} mins".format(current_stateOn,next_stateOn,dt_state['duration']))
        #dt_state = [next_stateOn, time.time(), 0]
        #dt_state.update({'state': next_stateOn,'start_ts': time.time(), 'duration': 0})
    elif current_stateOn==True and dt_state['duration'] > compOn_maxMinutes: #dt_state[2] > compOn_maxMinutes:
        # timeout compressor to OFF
        next_stateOn = False
        state_change = 3
        #logging.debug("3 - cur_state, next_state, dur_state : {} --> {} - {} mins".format(current_stateOn,next_stateOn,dt_state['duration']))
        #dt_state = [next_stateOn, time.time(), 0]
        #dt_state.update({'state': next_stateOn,'start_ts': time.time(), 'duration': 0})
    #else:
        # no state change
        #next_stateOn = current_stateOn
        # outputs
            # next_stateOn - state of compressor (ON/OFF) for next cycle
            # dt_state = [next_stateOn, current timestamp - duration in minutes
    #return next_stateOn, dt_state

    # tasks if an event occurred
    if current_stateOn != next_stateOn:
        # update dt_state
        dt_state.update({'state': next_stateOn,'start_ts': time.time(), 'duration': 0})
        # write to log
        logging.debug(f"{state_change} - cur_state, next_state, dur_state : {current_stateOn} --> {next_stateOn} - {current_stateDuration} mins")
        # publish event over mqtt
        eventData = {'event_type':'compState',
            'event_current':next_stateOn,
            'event_last': current_stateOn,
            'lastDuration': current_stateDuration,
            'ts': dt_state['start_ts']}
        mqtt_pub(client, 'event', eventData)

    return dt_state

def log_data(sysData): 
    logging.debug(f"""intTemp, next_state, state, state_ts, state_duration : ({sysData['intTemp']}, {sysData['next_state']}, {sysData['state']}, {sysData['state_ts']}, {sysData['state_duration']})""")

def read_sensors():
    # temperature sensors
    _,intTemp = read_temp(device_files['28-0000065be5ef'])
    _,extTemp = read_temp(device_files['28-0000065c181d'])
    _,compTemp = read_temp(device_files['28-0000065cd66d'])
    # current transducer
    compAmps = -1
    # keg weight
    kegWeight = -1
    # door state
    doorState = "CLOSED"

    # compile values into dict
    sensor_dict = {'intTemp' : intTemp,
                'extTemp' : extTemp,
                'compTemp': compTemp,
                'compAmps': compAmps,
                'kegWeight': kegWeight,
                'doorState': doorState,
                'ts': time.time_ns()}
    return sensor_dict

def mqtt_pub(client, msg_type, data):
    # publish sensor, event, and config data to mqtt broker
    # client - established client to mqtt broker
    # msg_type - ['sensor', 'event', 'config']
    # data - dictionary of data related to msg_type
    if msg_type == 'sensor':
        # sensor data pub
        topic = 'kegerator/sensor'
        msg = msgSensor_format.format(_intTemp=data['intTemp'],
                                _extTemp=data['extTemp'],
                                _compTemp=data['compTemp'],
                                _compAmps=data['compAmps'],
                                _kegWeight=data['kegWeight'],
                                _compState=data['compState'],
                                _doorState=data['doorState'],
                                _ts=data['ts'])
    if msg_type == 'event':
        # event data pub
        topic = 'kegerator/event'
        msg = msgEvent_format.format(_type=data['event_type'],
                                _current=data['event_current'],
                                _last=data['event_last'],
                                _duration=data['lastDuration'],
                                _ts=data['ts'])
    if msg_type == 'config':
        # config data pub
        topic = 'kegerator/config'
        msg = msgConfig_format.format(_compOffTemp=data['coolingOff_temp'],
                                _compOnTemp=data['coolingOn_temp'],
                                _compOnMinTime=data['compOn_minMinutes'],
                                _compOnMaxTime=data['compOn_maxMinutes'],
                                _compOffMinTime=data['compOff_minMinutes'],
                                _ts=data['ts'])
    # publish message
    result = client.publish(topic, msg)
    status = result[0]

    if status == 0:
        logging.debug(f"Sent {msg_type} data to '{topic}'")
    else:
        logging.debug(f"Failed to send {msg_type} data to topic '{topic}'")


def main():
    # set inital last state of compressor to OFF
    #current_stateOn = False
    #dt_state = [False,0,0] # (current_state, next_state,start_time,duration), initialize tracker for time in state
    dt_state = {'state': False,
                'start_ts': 0,
                'duration': 0}
    # mqtt setup
    client = connect_mqtt()
    client.loop_start()
    logging.info("connection to mqtt broker success")
    
    # publish config message
    mqtt_pub(client, 'config', configData) 

    # runtime code
    while True:
        # read sensors
        #_,intTemp = read_temp(device_files['28-0000065be5ef'])
        #_,extTemp = read_temp(device_files['28-0000065c181d'])
        #_,compTemp = read_temp(device_files['28-0000065cd66d'])
        sysData = read_sensors()
        
        intTemp = sysData['intTemp']
        #DEBUG intTemp = float(input("Enter intTemp : ")) #DEBUG
        
        current_stateOn = dt_state['state']
        # determine compressor state
        #compOn, dt_state = thermostat(intTemp,current_stateOn, dt_state) #TODO - simplify by removing current_stateOn for thermostat()
        dt_state = thermostat(intTemp, dt_state, client)
        compOn = dt_state['state']
        # update sensor_dict for mqtt pub message
        sysData.update({'compState': 'ON' if compOn else 'OFF',
                            'next_state': compOn,
                            'state': current_stateOn,
                            'state_ts': round(dt_state['start_ts'],1),
                            'state_duration': round(dt_state['duration'],1)})

        # log
        log_data(sysData)
        #logging.debug("intTemp, next_state, state, state_ts, state_duration is ({}, {}, {}, {}, {})".format(intTemp,compOn,current_stateOn,round(dt_state[1],1),round(dt_state[2],1)))
        
        # change compressor state
        comp_switch(compOn)
        # update dt_state for new compressor state
        #dt_state['state'] = compOn
        
        # publish sensor to mqtt
        mqtt_pub(client, 'sensor', sysData)

        time.sleep(1)

if __name__ == "__main__":
    ## initialization
    init_error = initialize()
    # main program
    main()
