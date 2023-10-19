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
    c2c_traffic_receiver_test = C2CTrafficReceiverTest()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_TRAFFIC_RECEIVER_TEST',
        'plugin_type': 'sim'
    }
    return config

class C2CTrafficReceiverTest(Entity):
    def __init__(self):
        super().__init__()

        self.lock = threading.Lock()

        self.mqtt_client = MQTTC2CTrafficClient()
        self.mqtt_client.run()

        # Traffic variables of test object(s)
        self.ac_id = ['test_1']
        self.vn = [5.]
        self.ve = [0.]
        self.vd = [0.]
        self.lat = [52.3]
        self.lon = [4.7]
        self.alt = [40.]

    @timed_function(dt=0.1)
    def test_c2c_traffic_receiver(self):
        for i in range(len(self.ac_id)):
            message_dict = {"ac_id" : self.ac_id[i],\
                            "lat" : self.lat[i],\
                            "lon" : self.lon[i],\
                            "alt" : self.alt[i],\
                            "vn" : self.vn[i],\
                            "ve" : self.ve[i],\
                            "vd" : self.vd[i]}
            self.mqtt_client.publish('daa/traffic', json.dumps(message_dict))

            # Update lat lon alt
            self.lat[i] = self.lat[i]
            self.lon[i] = self.lon[i]
            self.alt[i] = self.alt[i]

class MQTTC2CTrafficClient(mqtt.Client):
    def __init__(self):
        super().__init__()

    def run(self):
        self.connect(os.environ["MQTT_HOST"], 1883, 60)
        rc = self.loop_start()
        return rc

    def on_message(self, mqttc, obj, msg):
        return

    def stop(self):
        self.loop_stop()

