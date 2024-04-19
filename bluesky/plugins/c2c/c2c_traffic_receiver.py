import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import stack

import paho.mqtt.client as mqtt
import threading

import numpy as np

import json
import time

import os

c2c_traffic_receiver = None

def init_plugin():
    # Instantiate C2CTraffic entity
    global c2c_traffic_receiver
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
        # List of mqtt msgs copied from the buffer
        self.mqtt_msgs = []

        # struct of traffic objects
        self.traffic_objects = {}

        # Start mqtt client to read out control commands
        self.mqtt_client = MQTTC2CTrafficReceiverClient(self)
        self.mqtt_client.run()

    def recv_mqtt(self, msg):
        self.lock.acquire()
        try:
            if msg.topic == 'daa/traffic': 
                self.mqtt_msg_buf.append(json.loads(msg.payload))
        finally:
            self.lock.release()

    def copy_buffers(self):
        self.lock.acquire()
        try:
            for msg in self.mqtt_msg_buf:
                self.mqtt_msgs.append(msg)

            # Empty buffers
            self.mqtt_msg_buf = []
        finally:
            self.lock.release()

    def update_traffic_object(self, msg):
        # Check if traffic already exists
        if str(msg['ac_id']) in self.traffic_objects.keys():
            self.traffic_objects[str(msg['ac_id'])].update(msg)
        else:
            self.traffic_objects[str(msg['ac_id'])] = C2CTraffic(msg)

    @timed_function(dt=0.05)
    def update_c2c_traffic(self):
        self.copy_buffers()
        # Read new messages from buffer
        for msg in self.mqtt_msgs:
            self.update_traffic_object(msg)

        # Empty msgs
        self.mqtt_msgs = []

        # Check non updated traffic
        time_now_s = time.time()
        remove_keys = []
        for key in self.traffic_objects.keys():
            # Delete if nothing received for 10 seconds
            if (time_now_s - self.traffic_objects[key].timestamp_s) > 10.:
                self.traffic_objects[key].remove()
                remove_keys.append(key)
        
        for key in remove_keys:
            self.traffic_objects.pop(key)
    
class C2CTraffic(object):
    def __init__(self, msg):
        self.ac_id = str(msg['ac_id'])
        self.lat = float(msg['lat']) / 10**7
        self.lon = float(msg['lon']) / 10**7
        self.alt = float(msg['alt']) / 10**3
        self.vn = float(msg['vn']) / 10**3
        self.ve = float(msg['ve']) / 10**3
        self.vd = float(msg['vd']) / 10**3
        self.hdg = np.rad2deg(np.arctan2(self.ve, self.vn))
        self.h_spd = np.sqrt(self.ve**2 + self.vn**2)
        self.timestamp_s = time.time()
        if self.h_spd < 0.1:
            self.h_spd = 0.
        bs.traf.cre(self.ac_id, 'MAVIC', self.lat, self.lon, self.hdg, self.alt, self.h_spd)
    
    def update(self, msg):
        self.lat = float(msg['lat']) / 10**7
        self.lon = float(msg['lon']) / 10**7
        self.alt = float(msg['alt']) / 10**3
        self.vn = float(msg['vn']) / 10**3
        self.ve = float(msg['ve']) / 10**3
        self.vd = float(msg['vd']) / 10**3
        self.hdg = np.rad2deg(np.arctan2(self.ve, self.vn))
        self.h_spd = np.sqrt(self.ve**2 + self.vn**2)
        self.timestamp_s = time.time()
        if self.h_spd < 0.1:
            self.h_spd = 0.
        bs.traf.move(bs.traf.id2idx(self.ac_id), self.lat, self.lon, self.alt, self.hdg, self.h_spd, -self.vd)

    def remove(self):
        bs.traf.delete(bs.traf.id2idx(self.ac_id))

class MQTTC2CTrafficReceiverClient(mqtt.Client):
    def __init__(self, c2c_traffic_object):
        super().__init__()
        self.c2c_traffic_object = c2c_traffic_object

    def run(self):
        self.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
        self.subscribe("daa/traffic", 0)
        rc = self.loop_start()
        return rc

    def on_message(self, mqttc, obj, msg):
        self.c2c_traffic_object.recv_mqtt(msg)

    def stop(self):
        self.loop_stop()