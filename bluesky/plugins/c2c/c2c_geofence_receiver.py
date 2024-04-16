import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.tools import areafilter

import paho.mqtt.client as mqtt
import threading

import numpy as np

import json
import time

import os

def init_plugin():
    # Instantiate C2COwnstate entity
    c2c_geofence_receiver = C2CGeofenceReceiver()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_GEOFENCE_RECEIVER',
        'plugin_type': 'sim'
    }
    return config

class C2CGeofenceReceiver(Entity):
    def __init__(self):
        super().__init__()

        self.lock = threading.Lock()

        # Mqtt message buffer
        self.mqtt_msg_buf = []
        # List of mqtt msgs copied from the buffer
        self.mqtt_msgs = []

        # struct of ownstate objects
        self.geofence_objects = {}

        # Start mqtt client to read out control commands
        self.mqtt_client = MQTTC2CGeofenceReceiverClient(self)
        self.mqtt_client.run()
        
    def recv_mqtt(self, msg):
        self.lock.acquire()
        try:
            if msg.topic == 'daa/geofence': 
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

    def update_geofence_object(self, msg):
        # Check if geofence already assigned before
        if str(msg['ac_id']) in self.geofence_objects.keys():
            self.geofence_objects[str(msg['ac_id'])].update(msg)
        else:
            self.geofence_objects[str(msg['ac_id'])] = C2CGeofence(msg)
        

    @timed_function(dt=0.05)
    def update_c2c_geofence(self):
        self.copy_buffers()
        # Read new messages from buffer
        for msg in self.mqtt_msgs:
            self.update_geofence_object(msg)

        # Empty msgs
        self.mqtt_msgs = []

        # Check if geofences for not existing traffic
        remove_keys = []
        for key in self.geofence_objects.keys():
            # Check if key is in traffic callsign list
            if key not in bs.traf.id:
                remove_keys.append(key)
        
        for key in remove_keys:
            self.geofence_objects[key].delete()
            self.geofence_objects.pop(key)
    
class C2CGeofence(object):
    def __init__(self, msg):
        self.ac_id = str(msg['ac_id'])
        self.geozone = []
        self.timestamp_s = time.time()

        for i in range(len(msg['geozone'])):
            lat = float(msg['geozone'][i]['lat']) / 10**7 
            lon = float(msg['geozone'][i]['lon']) / 10**7 
            self.geozone.append(lat)
            self.geozone.append(lon)
        areafilter.defineArea('GF_' + str(self.ac_id), 'POLY', self.geozone)
    
    def update(self, msg):
        self.geozone = []
        self.timestamp_s = time.time()

        for i in range(len(msg['geozone'])):
            lat = float(msg['geozone'][i]['lat']) / 10**7 
            lon = float(msg['geozone'][i]['lon']) / 10**7 
            self.geozone.append(lat)
            self.geozone.append(lon)
        areafilter.deleteArea('GF_' + str(self.ac_id))
        areafilter.defineArea('GF_' + str(self.ac_id), 'POLY', self.geozone)

    def delete(self):
        areafilter.deleteArea('GF_' + str(self.ac_id))

class MQTTC2CGeofenceReceiverClient(mqtt.Client):
    def __init__(self, c2c_geofence_object):
        super().__init__()
        self.c2c_geofence_object = c2c_geofence_object

    def run(self):
        self.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
        self.subscribe("daa/geofence", 0)
        rc = self.loop_start()
        return rc

    def on_message(self, mqttc, obj, msg):
        self.c2c_geofence_object.recv_mqtt(msg)

    def stop(self):
        self.loop_stop()