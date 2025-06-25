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
        logger.debug("Traffic Publisher MQTT client published message with mid: %(mid)s", {'mid': str(mid)}) # ?? improve
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
        
        # eprint("Number of bluesky traffic objects: ", bs.traf.ntraf)

        # Publish traffic not received by c2ctrafficreceiver
        if ((traf_receiver.c2c_traffic_receiver is not None) and (ownstate_receiver.c2c_ownstate_receiver is not None)):
            # Upper cased keys
            upper_keys = []
            for key in traf_receiver.c2c_traffic_receiver.traffic_objects.keys():
                upper_keys.append(key.upper())

            for key in ownstate_receiver.c2c_ownstate_receiver.ownstate_objects.keys():
                upper_keys.append(key.upper())
            
            # For debugging
            # eprint("Bluesky Traffic ids:")
            # eprint(bs.traf.id)
            # eprint("Traffic and Ownstate keys:")
            # eprint(upper_keys)

            for i in range(bs.traf.ntraf):
                if bs.traf.id[i] not in upper_keys:
                    # send resolution over mqtt
                    body = {}
                    body['ac_id'] = str(bs.traf.id[i])
                    body['lat'] = int(bs.traf.lat[i] * 10**7)
                    body['lon'] = int(bs.traf.lon[i] * 10**7)
                    body['alt'] = int(bs.traf.alt[i] * 10**3)


                    body['vn'] = int(bs.traf.gsnorth[i] * 10**3)
                    body['ve'] = int(bs.traf.gseast[i] * 10**3)
                    body['vd'] = int(-bs.traf.vs[i] * 10**3)

                    self.mqtt_client.publish('daa/traffic_out', payload=json.dumps(body))
        # Send all     
        else:
            for i in range(bs.traf.ntraf):
                # send resolution over mqtt
                body = {}
                body['ac_id'] = str(bs.traf.id[i])
                body['lat'] = int(bs.traf.lat[i] * 10**7)
                body['lon'] = int(bs.traf.lon[i] * 10**7)
                body['alt'] = int(bs.traf.alt[i] * 10**3)


                body['vn'] = int(bs.traf.gsnorth[i] * 10**3)
                body['ve'] = int(bs.traf.gseast[i] * 10**3)
                body['vd'] = int(-bs.traf.vs[i] * 10**3)

                self.mqtt_client.publish('daa/traffic_out', payload=json.dumps(body))
        return


