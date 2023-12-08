import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky import stack
from bluesky.tools import geo
from bluesky.tools.aero import nm, ft

import paho.mqtt.client as mqtt
import threading

import numpy as np

import json
import time

import os

class MQTTAvoidRequestPublisher(mqtt.Client):

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

mqtt_publisher = MQTTAvoidRequestPublisher(mqtt.Client)
mqtt_publisher.connect(os.environ["MQTT_HOST"])

def init_plugin():
    # Instantiate C2COwnstate entity
    c2c_avoid_test = C2CAvoidTest()
    # Configuration parameters
    config = {
        'plugin_name': 'C2C_AVOID_TEST',
        'plugin_type': 'sim'
    }
    return config

class C2CAvoidTest(Entity):
    def __init__(self):
        super().__init__()


@stack.command()
def generate_testresolution(acid: str):
    if acid in bs.traf.id:
        i = bs.traf.id[acid]
        qdr_res = 45. #[deg]
        dist_res = 100. / nm # [nm]
        lat_res, lon_res = geo.qdrpos(bs.traf.lat[i], bs.traf.lon[i], qdr_res, dist_res)
        alt_res = bs.traf.alt[i] # [m]

        # Send request
        mqtt_publisher.loop_start()
        body = {}
        body['timestamp'] = int(time.time())
        body['waypoint'] = {}
        body['waypoint']['lat'] = lat_res * 10**7
        body['waypoint']['lon'] = lon_res * 10**7
        body['alt'] = alt_res * 10**3
        
        infot = mqtt_publisher.publish('daa/avoid_request', payload=json.dumps(body))
        #infot.wait_for_publish()
        mqtt_publisher.loop_stop()

    else:
        return