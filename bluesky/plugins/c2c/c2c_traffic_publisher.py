import bluesky as bs
from bluesky.core import Entity, timed_function

import paho.mqtt.client as mqtt
import numpy as np
import bluesky.plugins.c2c.c2c_traffic_receiver as traf_receiver
import bluesky.plugins.c2c.c2c_ownstate_receiver as ownstate_receiver
import time
import json

import os
import logging
import logging.config
class UTCFormatter(logging.Formatter):
    converter = time.gmtime

with open('../logging_c2c.json', 'r') as f:
    config = json.load(f)

logging.config.dictConfig(config)
logger = logging.getLogger("traffic_publisher")

c2c_traffic_publisher_loop_flag = 1

def init_plugin():
    # Instantiate C2CTraffic entity
    c2c_traffic_publisher = C2CTrafficPublisher()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_TRAFFIC_PUBLISHER',
        'plugin_type': 'sim'
    }

    logger.info("C2C_TRAFFIC_PUBLISHER plugin initialized")
    
    return config

class MQTTC2CTrafficPublisher(mqtt.Client):

    def __init__(self, c2c_traffic_object):
        super().__init__()
        self.c2c_traffic_object = c2c_traffic_object
        self._publish_cache = {}  # Cache to track what was published

    def run(self):
        # Make Traffic publisher MQTT client
        self.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
        self.loop_start()

        while c2c_traffic_publisher_loop_flag == 1:
            logger.debug("Waiting for Traffic Publisher MQTT client to connect...")
            time.sleep(0.1)

    def on_connect(self, mqttc, obj, flags, rc):
        global c2c_traffic_publisher_loop_flag
        c2c_traffic_publisher_loop_flag = 0
        logger.info("Traffic Publisher MQTT client connect with result code: %(code)s", {'code': mqtt.error_string(rc)})
        return

    def on_message(self, mqttc, obj, msg):
        return

    def on_publish(self, mqttc, obj, mid):
        if logger.isEnabledFor(logging.DEBUG):
            # Retrieve cached message info if available
            msg_info = self._publish_cache.pop(mid, None)
            if msg_info:
                logger.debug("Traffic published: topic=%s ac_id=%s lat=%d lon=%d alt=%d mid=%d",
                           msg_info['topic'], msg_info['ac_id'], msg_info['lat'], 
                           msg_info['lon'], msg_info['alt'], mid)
            else:
                logger.debug("Traffic Publisher MQTT client published message with mid: %d", mid)
        return

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        return

    def on_log(self, mqttc, obj, level, string):
        return
    
    def stop(self):
        logger.info("Stopping Traffic Publisher MQTT client")
        self.loop_stop()

class C2CTrafficPublisher(Entity):
    def __init__(self):
        super().__init__()
        # Start mqtt client to read out control commands
        self.mqtt_client = MQTTC2CTrafficPublisher(self)
        self.mqtt_client.run()
        
    @timed_function(dt=0.2)
    def publish_c2c_traffic(self):
        
        # Publish traffic not received by c2ctrafficreceiver
        if ((traf_receiver.c2c_traffic_receiver is not None) and (ownstate_receiver.c2c_ownstate_receiver is not None)):
            upper_keys = {k.upper() for k in traf_receiver.c2c_traffic_receiver.traffic_objects.keys()}
            upper_keys |= {k.upper() for k in ownstate_receiver.c2c_ownstate_receiver.ownstate_objects.keys()}

            ntraf = bs.traf.ntraf
            if ntraf == 0:
                return
            ids = bs.traf.id
            lat = bs.traf.lat
            lon = bs.traf.lon
            alt = bs.traf.alt
            gsn = bs.traf.gsnorth
            gse = bs.traf.gseast
            vs = bs.traf.vs

            for i in range(ntraf):
                if str(ids[i]).upper() not in upper_keys:
                    # send resolution over mqtt
                    body = {
                        'ac_id': str(ids[i]),
                        'lat': int(lat[i] * 10**7),
                        'lon': int(lon[i] * 10**7),
                        'alt': int(alt[i] * 10**3),
                        'vn': int(gsn[i] * 10**3),
                        've': int(gse[i] * 10**3),
                        'vd': int(-vs[i] * 10**3),
                    }

                    msg_info = self.mqtt_client.publish('daa/traffic_out', payload=json.dumps(body))
                    # Cache message info for debug logging
                    if logger.isEnabledFor(logging.DEBUG) and msg_info.rc == mqtt.MQTT_ERR_SUCCESS:
                        self.mqtt_client._publish_cache[msg_info.mid] = {
                            'topic': 'daa/traffic_out',
                            'ac_id': body['ac_id'],
                            'lat': body['lat'],
                            'lon': body['lon'],
                            'alt': body['alt']
                        }
        # Send all     
        else:
            ntraf = bs.traf.ntraf
            if ntraf == 0:
                return
            ids = bs.traf.id
            lat = bs.traf.lat
            lon = bs.traf.lon
            alt = bs.traf.alt
            gsn = bs.traf.gsnorth
            gse = bs.traf.gseast
            vs = bs.traf.vs
            for i in range(ntraf):
                # send resolution over mqtt
                body = {
                    'ac_id': str(ids[i]),
                    'lat': int(lat[i] * 10**7),
                    'lon': int(lon[i] * 10**7),
                    'alt': int(alt[i] * 10**3),
                    'vn': int(gsn[i] * 10**3),
                    've': int(gse[i] * 10**3),
                    'vd': int(-vs[i] * 10**3),
                }

                msg_info = self.mqtt_client.publish('daa/traffic_out', payload=json.dumps(body))
                # Cache message info for debug logging
                if logger.isEnabledFor(logging.DEBUG) and msg_info.rc == mqtt.MQTT_ERR_SUCCESS:
                    self.mqtt_client._publish_cache[msg_info.mid] = {
                        'topic': 'daa/traffic_out',
                        'ac_id': body['ac_id'],
                        'lat': body['lat'],
                        'lon': body['lon'],
                        'alt': body['alt']
                    }
        return


