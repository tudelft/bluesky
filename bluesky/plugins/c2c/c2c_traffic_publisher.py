import bluesky as bs
from bluesky.core import Entity, timed_function

import paho.mqtt.client as mqtt
import numpy as np
import bluesky.plugins.c2c.c2c_traffic_receiver as traf_receiver

import json

import os

def init_plugin():
    # Instantiate C2CTraffic entity
    c2c_traffic_publisher = C2CTrafficPublisher()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_TRAFFIC_PUBLISHER',
        'plugin_type': 'sim'
    }
    return config

class MQTTC2CTrafficPublisher(mqtt.Client):

    def on_connect(self, mqttc, obj, flags, rc):
        return

    def on_message(self, mqttc, obj, msg):
        return

    def on_publish(self, mqttc, obj, mid):
        return

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        return

    def on_log(self, mqttc, obj, level, string):
        return

class C2CTrafficPublisher(Entity):
    def __init__(self):
        super().__init__()

    @timed_function(dt=0.2)
    def publish_c2c_traffic(self):
        
        # Publish traffic not received by c2ctrafficreceiver
        if (traf_receiver.c2c_traffic_receiver is not None):
            # Upper cased keys
            upper_keys = []
            for key in traf_receiver.c2c_traffic_receiver.traffic_objects.keys():
                upper_keys.append(key.upper())
                    
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

                    mqtt_publisher = MQTTC2CTrafficPublisher()
                    mqtt_publisher.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
                    mqtt_publisher.loop_start()
                    mqtt_publisher.publish('daa/traffic_out', payload=json.dumps(body))
                    mqtt_publisher.loop_stop()
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

                mqtt_publisher = MQTTC2CTrafficPublisher()
                mqtt_publisher.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
                mqtt_publisher.loop_start()
                mqtt_publisher.publish('daa/traffic_out', payload=json.dumps(body))
                mqtt_publisher.loop_stop()



