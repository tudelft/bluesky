import bluesky as bs
from bluesky.core import Entity, timed_function

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

# Try to load logging config from relative path, with fallback to absolute path
logging_config_path = '../logging_c2c.json'
if not os.path.exists(logging_config_path):
    # Try absolute path from bluesky root
    logging_config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'logging_c2c.json')

if os.path.exists(logging_config_path):
    with open(logging_config_path, 'r') as f:
        config = json.load(f)
    logging.config.dictConfig(config)
else:
    # Fallback to basic config if file not found
    logging.basicConfig(level=logging.INFO)

logger = logging.getLogger("ownstate_receiver")

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

    logger.info("C2C_OWNSTATE_RECEIVER plugin initialized")
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
        """Receive MQTT message; parse outside the lock to minimize contention."""
        if msg.topic != 'daa/ownstate':
            return

        try:
            data = json.loads(msg.payload)
        except Exception as e:
            logger.warning("Failed to parse ownstate payload: %(err)s", {'err': str(e)})
            return

        # Append under lock
        with self.lock:
            self.mqtt_msg_buf.append(data)

    def copy_buffers(self):
        """Move buffered MQTT messages to processing list with minimal copying."""
        with self.lock:
            if not self.mqtt_msg_buf:
                return
            if self.mqtt_msgs:
                # Extend existing processing list
                self.mqtt_msgs.extend(self.mqtt_msg_buf)
                self.mqtt_msg_buf = []
            else:
                # Swap lists to avoid O(n) extend
                self.mqtt_msgs, self.mqtt_msg_buf = self.mqtt_msg_buf, []

    def update_ownstate_object(self, msg):
        ac_id_str = str(msg.get('ac_id'))
        
        # Handle delete requests first (before validation)
        if msg.get('delete', False):
            # Remove from our tracking dict
            if ac_id_str in self.ownstate_objects:
                try:
                    self.ownstate_objects[ac_id_str].remove()
                except Exception as e:
                    logger.debug("Failed to remove aircraft %(ac_id)s: %(err)s", 
                                {'ac_id': ac_id_str, 'err': str(e)})
                finally:
                    self.ownstate_objects.pop(ac_id_str, None)
            return

        # Check if msg is valid
        if any(msg.get(k) is None for k in required_keys):
            logger.warning("Received invalid ownstate message: %(msg)s", {'msg': json.dumps(msg)})
            return
        
        try:
            # Check if ownstate already exists
            if ac_id_str in self.ownstate_objects:
                self.ownstate_objects[ac_id_str].update(msg)
            else:
                self.ownstate_objects[ac_id_str] = C2COwnstate(msg)
        except TypeError as e:
            logger.error("%(error)s encountered while updating ownstate object: %(msg)s", {'error': str(e), 'msg': json.dumps(msg)})
            # Clean up ownstate object in case it was modified
            try:
                self.ownstate_objects[ac_id_str].remove()
            except Exception:
                pass
            return
        

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
        # Collect expired keys in one pass over items to avoid repeated dict lookups
        remove_keys = [key for key, obj in self.ownstate_objects.items()
                       if (time_now_s - obj.timestamp_s) > 10.0]

        # Remove expired ownstate objects
        for key in remove_keys:
            try:
                self.ownstate_objects[key].remove()
            finally:
                self.ownstate_objects.pop(key, None)
    
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
        idx = bs.traf.id2idx(self.ac_id)
        # Handle both int and list return types from id2idx
        if isinstance(idx, (list, tuple)):
            if len(idx) > 0:
                idx = idx[0]
            else:
                return  # Aircraft not found
        if idx >= 0:
            bs.traf.move(idx, self.lat, self.lon, self.alt, self.hdg, self.h_spd, -self.vd)

    def remove(self):
        idx = bs.traf.id2idx(self.ac_id)
        # id2idx can return int or list; handle both cases
        if isinstance(idx, (list, tuple)):
            if len(idx) > 0 and idx[0] >= 0:
                bs.traf.delete(idx[0])
        elif idx >= 0:  # Single int index
            bs.traf.delete(idx)

class MQTTC2COwnstateReceiverClient(mqtt.Client):
    def __init__(self, c2c_ownstate_object):
        super().__init__()
        self.c2c_ownstate_object = c2c_ownstate_object

    def run(self):
        self.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
        rc = self.loop_start()
        while c2c_ownstate_receiver_loop_flag == 1:
            logger.debug("Waiting for Ownstate MQTT client to connect...")
            time.sleep(0.1)

        self.subscribe("daa/ownstate", 0)
        
        return rc

    def on_message(self, mqttc, obj, msg):
        if logger.isEnabledFor(logging.DEBUG):
            try:
                data = json.loads(msg.payload.decode('utf-8'))
                logger.debug("Ownstate received: topic=%s ac_id=%s lat=%d lon=%d alt=%d",
                           msg.topic, data.get('ac_id', 'unknown'),
                           data.get('lat', 0), data.get('lon', 0), data.get('alt', 0))
            except Exception as e:
                logger.debug("Ownstate Receiver MQTT client received message: topic=%s (parse error: %s)",
                           msg.topic, str(e))
        self.c2c_ownstate_object.recv_mqtt(msg)

    def on_connect(self, mqttc, obj, flags, rc):
        global c2c_ownstate_receiver_loop_flag
        c2c_ownstate_receiver_loop_flag = 0
        logger.info("Ownstate Receiver MQTT client connect with result code: %(code)s", {'code': mqtt.error_string(rc)})

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        return

    def stop(self):
        logger.info("Stopping Ownstate Receiver MQTT client...")
        self.loop_stop()
