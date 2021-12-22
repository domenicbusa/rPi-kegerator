# python 3.6

import random
import time

from paho.mqtt import client as mqtt_client


broker = '192.168.68.109'
port = 1883
topic0 = "kegerator/sensor"
topic1 = "kegerator/event"
topic2 = "kegerator/config"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
# username = 'emqx'
# password = 'public'

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client):
    msg_count = 0.1
    msg0_format = """{{"sensors": {{
                "intTemp": {_intTemp},
                "extTemp": {_extTemp},
                "compTemp": {_compTemp},
                "compAmps": {_compAmps},
                "kegWeight": {_kegWeight} }},
            "tags": {{
                "compState": "{_compState}",
                "doorState": "{_doorState}"}},
            "ts": {_ts} }}"""
    msg1_format = """{{"event_type": "{_type}",
            "event_current":"{_current}",
            "event_last": "{_last}",
            "lastDuration": {_duration},
            "ts": {_ts} }}"""
    msg2_format = """{{"compOffTemp": {_compOffTemp},
            "compOnTemp": {_compOnTemp},
            "compOnMinTime": {_compOnMinTime},
            "compOnMaxTime": {_compOnMaxTime},
            "compOffMinTime": {_compOffMinTime},
            "ts": {_ts} }}"""
    compressor_state = 'ON'
    last1_ts = 0
    last2_ts = 0
    event_timeout =  15000000000 # 15 seconds
    config_timeout = 30000000000 # 30 seconds
    while True:
        time.sleep(5)
        cur_ts = time.time_ns()
        compressor_state = "OFF" if compressor_state=="ON" else "ON"
        # define message content
        msg0 = msg0_format.format(_intTemp=msg_count,
                                _extTemp=msg_count+1.1,
                                _compTemp=msg_count+2.2,
                                _compAmps=msg_count+3.3,
                                _kegWeight=msg_count+4.4,
                                _compState=compressor_state,
                                _doorState="CLOSED",
                                _ts=cur_ts)
        msg1 = msg1_format.format(_type="compState",
                                _current="ON",
                                _last="OFF",
                                _duration=360,
                                _ts=cur_ts)
        msg2 = msg2_format.format(_compOffTemp=36,
                                _compOnTemp=42,
                                _compOnMinTime=5,
                                _compOnMaxTime=15,
                                _compOffMinTime=8,
                                _ts=cur_ts)
        
        # publish event data
        if cur_ts - last1_ts > event_timeout:
            # publish event data
            result1 = client.publish(topic1, msg1)
            last1_ts = cur_ts
            print(f"{cur_ts} --- Event : `{msg1}` to topic `{topic1}`")
        
        # publish config data
        if cur_ts - last2_ts > config_timeout:
            # publish config data
            result2 = client.publish(topic2, msg2)
            last2_ts = cur_ts
            print(f"{cur_ts} -- Config : `{msg2}` to topic `{topic2}`")
        
        # publish sensor data
        result0 = client.publish(topic0, msg0)
        status = result0[0]
        if status == 0:
            print(f"{cur_ts} --- Sensor : `{msg0}` to topic `{topic0}`\n------------------------\n")
        else:
            print(f"Failed to send message to topic {topic}")
            #TODO - Add a reconnect try
        
        msg_count += 1


def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    run()
