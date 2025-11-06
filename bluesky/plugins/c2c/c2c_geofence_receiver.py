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

import logging
import logging.config
class UTCFormatter(logging.Formatter):
    converter = time.gmtime

with open('../logging_c2c.json', 'r') as f:
    config = json.load(f)

logging.config.dictConfig(config)
logger = logging.getLogger("geofence_receiver")

c2c_geofence_receiver_loop_flag = 1

def init_plugin():
    # Instantiate C2COwnstate entity
    c2c_geofence_receiver = C2CGeofenceReceiver()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_GEOFENCE_RECEIVER',
        'plugin_type': 'sim'
    }

    logger.info("C2C_GEOFENCE_RECEIVER plugin initialized")

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
        """Receive MQTT message; parse outside the lock to minimize contention."""
        if msg.topic != 'daa/geofence':
            return
        try:
            payload = json.loads(msg.payload)
        except Exception as e:
            logger.warning("Failed to parse geofence payload: %(err)s", {'err': str(e)})
            return
        with self.lock:
            self.mqtt_msg_buf.append(payload)

    def copy_buffers(self):
        with self.lock:
            if not self.mqtt_msg_buf:
                return
            if self.mqtt_msgs:
                self.mqtt_msgs.extend(self.mqtt_msg_buf)
                self.mqtt_msg_buf = []
            else:
                self.mqtt_msgs, self.mqtt_msg_buf = self.mqtt_msg_buf, []

    def update_geofence_object(self, msg):
        # Check if geofence already assigned before
        ac_id_str = str(msg.get('ac_id'))
        if ac_id_str in self.geofence_objects:
            self.geofence_objects[ac_id_str].update(msg)
        else:
            self.geofence_objects[ac_id_str] = C2CGeofence(msg)
        

    @timed_function(dt=0.05)
    def update_c2c_geofence(self):
        self.copy_buffers()
        # Read new messages from buffer
        for msg in self.mqtt_msgs:
            self.update_geofence_object(msg)

        # Empty msgs
        self.mqtt_msgs = []

        # Check if geofences for not existing traffic
        # Use a set for faster membership testing if bs.traf.id is large
        traf_ids = set(map(str, bs.traf.id)) if len(bs.traf.id) else set()
        remove_keys = [key for key in self.geofence_objects.keys() if key not in traf_ids]
        for key in remove_keys:
            try:
                self.geofence_objects[key].delete()
            finally:
                self.geofence_objects.pop(key, None)
    
class C2CGeofence(object):
    def __init__(self, msg):
        self.ac_id = str(msg['ac_id'])
        self.geozone = []
        self.timestamp_s = time.time()
        # Build geozone list with list comprehension
        gz = msg.get('geozone', [])
        self.geozone = [coord for pt in gz for coord in (float(pt['lat'])/10**7, float(pt['lon'])/10**7)]
        
        area_created = areafilter.defineArea('GF_' + str(self.ac_id), 'POLY', self.geozone)

        # if area_created:
        #     # Debug information
        #     print(str(self.ac_id) + " has a defined Geofence area: " + str(areafilter.hasArea('GF_' + str(self.ac_id))) + ", with type: " + area_type)
        #     print(str(self.ac_id) + " geofence has the following waypoints: ")
        #     for i in range(0, len(self.geozone), 2):
        #         print("Lat: " + str(self.geozone[i]) + ", Lon: " + str(self.geozone[i+1]))
        # else:
        #     print("Geofence creation for " + str(self.ac_id) + " failed with error: " + area_type)
    
    def update(self, msg):
        self.timestamp_s = time.time()
        gz = msg.get('geozone', [])
        self.geozone = [coord for pt in gz for coord in (float(pt['lat'])/10**7, float(pt['lon'])/10**7)]
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
        rc = self.loop_start()

        while c2c_geofence_receiver_loop_flag == 1:
            logger.debug("Waiting for Geofence Receiver MQTT client to connect...")
            time.sleep(0.1)

        self.subscribe("daa/geofence", 0)
        return rc

    def on_message(self, mqttc, obj, msg):
        if logger.isEnabledFor(logging.DEBUG):
            try:
                data = json.loads(msg.payload.decode('utf-8'))
                geozone = data.get('geozone', [])
                logger.debug("Geofence received: topic=%s ac_id=%s num_points=%d",
                           msg.topic, data.get('ac_id', 'unknown'), len(geozone))
            except Exception as e:
                logger.debug("Geofence Receiver MQTT client received message: topic=%s (parse error: %s)",
                           msg.topic, str(e))
        self.c2c_geofence_object.recv_mqtt(msg)

    def on_connect(self, mqttc, obj, flags, rc):
        global c2c_geofence_receiver_loop_flag
        c2c_geofence_receiver_loop_flag = 0
        logger.info("Geofence Receiver MQTT client connect with result code: %(code)s", {'code': mqtt.error_string(rc)})

    def stop(self):
        logger.info("Stopping Geofence Receiver MQTT client...")
        self.loop_stop()