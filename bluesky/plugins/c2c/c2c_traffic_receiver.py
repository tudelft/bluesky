import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import stack

import paho.mqtt.client as mqtt
import threading

import numpy as np

import json

import os

def init_plugin():
    # Instantiate C2CTraffic entity
    c2c_traffic_receiver = C2CTrafficReceiver()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_TRAFFIC_RECEIVER',
        'plugin_type': 'sim'
    }
    return config

class C2CTrafficReceiver(Entity):
    def __init__(self):
        super().__init__()

        self.lock = threading.Lock()

        # Mqtt message buffer
        self.mqtt_msg_buf = []

        # Start mqtt client to read out control commands
        self.mqtt_client = MQTTC2CTrafficReceiverClient(self)
        self.mqtt_client.run()

    def recv_mqtt(self, msg):
        self.lock.acquire()
        try:
            print(msg.topic)
            if msg.topic == 'daa/traffic': 
                self.mqtt_msg_buf.append(json.loads(msg.payload))
                print(json.loads(msg.payload))
        finally:
            self.lock.release()

    @timed_function(dt=0.05)
    def update_c2c_traffic(self):
        # Read message buffer to read new messages and empty
        self.mqtt_msg_buf = []
        return

class MQTTC2CTrafficReceiverClient(mqtt.Client):
    def __init__(self, c2c_traffic_object):
        super().__init__()
        self.c2c_traffic_object = c2c_traffic_object

    def run(self):
        self.connect(os.environ["MQTT_HOST"], os.environ["MQTT_PORT"], 60)
        self.subscribe("daa/traffic", 0)
        rc = self.loop_start()
        return rc

    def on_message(self, mqttc, obj, msg):
        self.c2c_traffic_object.recv_mqtt(msg)

    def stop(self):
        self.loop_stop()