import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.traffic.asas import ConflictResolution

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
logger = logging.getLogger("traffic_receiver")

c2c_traffic_receiver = None
c2c_traffic_receiver_loop_flag = 1

def init_plugin():
    # Instantiate C2CTraffic entity
    global c2c_traffic_receiver
    c2c_traffic_receiver = C2CTrafficReceiver()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_TRAFFIC_RECEIVER',
        'plugin_type': 'sim'
    }
    
    logger.info("C2C_TRAFFIC_RECEIVER plugin initialized")
    
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
        """Receive MQTT message; parse outside the lock to minimize contention."""
        if msg.topic != 'daa/traffic':
            return
        try:
            parsed_msg = json.loads(msg.payload)
        except Exception as e:
            logger.warning("Failed to parse traffic payload: %(err)s", {'err': str(e)})
            return
        # Append under lock
        with self.lock:
            self.mqtt_msg_buf.append(parsed_msg)

    def copy_buffers(self):
        """Move buffered MQTT messages to processing list with minimal copying."""
        with self.lock:
            if not self.mqtt_msg_buf:
                return
            if self.mqtt_msgs:
                self.mqtt_msgs.extend(self.mqtt_msg_buf)
                self.mqtt_msg_buf = []
            else:
                self.mqtt_msgs, self.mqtt_msg_buf = self.mqtt_msg_buf, []

    def update_traffic_object(self, msg):
        # Check if traffic already exists
        ac_id_str = str(msg.get('ac_id'))
        if ac_id_str in self.traffic_objects:
            self.traffic_objects[ac_id_str].update(msg)
        else:
            self.traffic_objects[ac_id_str] = C2CTraffic(msg)

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
        remove_keys = [key for key, obj in self.traffic_objects.items()
                       if (time_now_s - obj.timestamp_s) > 10.0]

        for key in remove_keys:
            try:
                self.traffic_objects[key].remove()
            finally:
                self.traffic_objects.pop(key, None)
    
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
        rc = self.loop_start()
        while c2c_traffic_receiver_loop_flag == 1:
            logger.debug("Waiting for Traffic Receiver MQTT client to connect...")
            time.sleep(0.1)
        
        self.subscribe("daa/traffic", 0)
        return rc

    def on_message(self, mqttc, obj, msg):
        if logger.isEnabledFor(logging.DEBUG):
            try:
                data = json.loads(msg.payload.decode('utf-8'))
                logger.debug("Traffic received: topic=%s ac_id=%s lat=%d lon=%d alt=%d",
                           msg.topic, data.get('ac_id', 'unknown'), 
                           data.get('lat', 0), data.get('lon', 0), data.get('alt', 0))
            except Exception as e:
                logger.debug("Traffic Receiver MQTT client received message: topic=%s (parse error: %s)", 
                           msg.topic, str(e))
        self.c2c_traffic_object.recv_mqtt(msg)

    def on_connect(self, mqttc, obj, flags, rc):
        global c2c_traffic_receiver_loop_flag
        c2c_traffic_receiver_loop_flag = 0
        logger.info("Traffic Receiver MQTT client connect with result code: %(code)s", {'code': mqtt.error_string(rc)})

    def stop(self):
        logger.info("Stopping Traffic Receiver MQTT client")
        self.loop_stop()