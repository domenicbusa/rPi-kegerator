import glob
import time
import RPi.GPIO as GPIO
import logging

# configuration
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'
pin_compSwitch = 26 # GPIO Output - switching the frige compressor on
pin_doorSensor = 19 # GPIO Input - determine if door is open
coolingOn_temp = 80
coolingOff_temp = 75
compOn_minMinutes = 2
compOn_maxMinutes = 15
compOff_minMinutes = 10

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
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_compSwitch, GPIO.OUT)
        GPIO.setup(pin_doorSensor, GPIO.IN)
        logging.info("init success")
        # TODO - insert log entry for configuration
    except:
        logging.error("ERROR in init")

def comp_switch(compOn):
    if compOn :
        GPIO.output(pin_compSwitch,GPIO.HIGH)
    else :
        GPIO.output(pin_compSwitch,GPIO.LOW)

def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def read_temp():
    lines = read_temp_raw()
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
    if T > coolingOn_temp and ~current_stateOn:
        if dt_state[2] > compOff_minMinutes:
            # compressor to ON
            next_stateOn = True
            dt_state = [next_stateOn, time.time(), 0]
        else:
            next_stateOn = current_stateOn
    elif T < coolingOff_temp and current_stateOn:
        if dt_state[2] > compOn_minMinutes:
            next_stateOn = False
            dt_state = [next_stateOn, time.time(), 0]
    elif dt_state[0] and dt_state[2] > compOn_maxMinutes:
        next_stateOn = False
        dt_state = [next_stateOn, time.time(), 0]
    else:
        next_stateOn = current_stateOn
    return next_stateOn, dt_state

def main():
    # set inital last state of compressor to OFF
    current_stateOn = False
    dt_state = [False,0,0] # (state,start_time,duration), initialize tracker for time in state

    # runtime code
    while True:
        # read temperature from sensor
        Tc,Tf = read_temp()
        # determine compressor state
        compOn, dt_state = thermostat(Tf,current_stateOn, dt_state)
        # change compressor state
        comp_switch(compOn)
        logging.debug("TempF, compOn, current_stateOn is ({}, {}, {})".format(Tf,compOn,current_stateOn))
        # save current cycle's compressor state for next cycle
        current_stateOn = compOn
        
        time.sleep(5)

if __name__ == "__main__":
    ## initialization
    init_error = initialize()
    # main program
    main()
