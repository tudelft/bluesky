import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import stack

import paho.mqtt.client as mqtt
import threading

import numpy as np

import json
import time

import os
import sys
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

required_keys = ['ac_id', 'lat', 'lon', 'alt', 'vn', 've', 'vd']
c2c_ownstate_receiver = None
c2c_ownstate_receiver_loop_flag = 1

def init_plugin():
    # Instantiate C2COwnstate entity
    global c2c_ownstate_receiver
    c2c_ownstate_receiver = C2COwnstateReceiver()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_OWNSTATE_RECEIVER',
        'plugin_type': 'sim'
    }

    if bs.settings.MQTT_debug:
        eprint("C2C Ownstate Receiver plugin loaded")
    
    return config

class C2COwnstateReceiver(Entity):
    def __init__(self):
        super().__init__()

        self.lock = threading.Lock()

        # Mqtt message buffer
        self.mqtt_msg_buf = []
        # List of mqtt msgs copied from the buffer
        self.mqtt_msgs = []

        # struct of ownstate objects
        self.ownstate_objects = {}

        # Start mqtt client to read out control commands
        self.mqtt_client = MQTTC2COwnstateReceiverClient(self)
        self.mqtt_client.run()

    def recv_mqtt(self, msg):
        self.lock.acquire()
        try:
            if msg.topic == 'daa/ownstate': 
                self.mqtt_msg_buf.append(json.loads(msg.payload))
        finally:
            self.lock.release()

    def copy_buffers(self):
        self.lock.acquire()
        try:
            self.mqtt_msgs.extend(self.mqtt_msg_buf)

            # Empty buffers
            self.mqtt_msg_buf = []
        finally:
            self.lock.release()

    def update_ownstate_object(self, msg):

        # Check if msg is valid
        # if not all(key in msg for key in required_keys) and None not in msg.values():
        #     if bs.settings.MQTT_debug:
        #         eprint("Received invalid or incomplete ownstate message, skipping...")
        #     return

        # Check if ownstate already exists
        if str(msg['ac_id']) in self.ownstate_objects.keys():
            self.ownstate_objects[str(msg['ac_id'])].update(msg)
        else:
            self.ownstate_objects[str(msg['ac_id'])] = C2COwnstate(msg)
        

    @timed_function(dt=0.05)
    def update_c2c_ownstate(self):
        self.copy_buffers()
        # Read new messages from buffer
        for msg in self.mqtt_msgs:
            self.update_ownstate_object(msg)

        # Empty msgs
        self.mqtt_msgs = []

        # Check non updated traffic
        time_now_s = time.time()
        remove_keys = []
        for key in self.ownstate_objects.keys():
            # Delete if nothing received for 10 seconds
            if (time_now_s - self.ownstate_objects[key].timestamp_s) > 10.:
                self.ownstate_objects[key].remove()
                remove_keys.append(key)
        
        for key in remove_keys:
            self.ownstate_objects.pop(key)
    
class C2COwnstate(object):
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

class MQTTC2COwnstateReceiverClient(mqtt.Client):
    def __init__(self, c2c_ownstate_object):
        super().__init__()
        self.c2c_ownstate_object = c2c_ownstate_object

    def run(self):
        self.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
        rc = self.loop_start()
        while c2c_ownstate_receiver_loop_flag == 1:
            eprint("Waiting for Ownstate MQTT client to connect...")
            time.sleep(0.1)

        self.subscribe("daa/ownstate", 0)
        
        return rc

    def on_message(self, mqttc, obj, msg):
        if bs.settings.MQTT_debug:
            eprint("Ownstate Receiver MQTT client received message: ", msg.topic, msg.payload)
        self.c2c_ownstate_object.recv_mqtt(msg)

    def on_connect(self, mqttc, obj, flags, rc):
        global c2c_ownstate_receiver_loop_flag
        c2c_ownstate_receiver_loop_flag = 0
        if bs.settings.MQTT_debug:
            eprint("Ownstate Receiver MQTT client connected with result code: ", mqtt.error_string(rc))

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        if bs.settings.MQTT_debug:
            eprint("Ownstate Receiver subscribed to: ", mid, " , with QoS level: ", granted_qos) # No idea how to get useful info from mid
        return

    def stop(self):
        self.loop_stop()
