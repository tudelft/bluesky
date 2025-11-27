"""
BlueSky Health Monitor Plugin

Publishes periodic heartbeats over MQTT and uses MQTT Last Will & Testament (LWT)
so external systems can detect crashes or stalls.

- Topic: bluesky/status (retained) -> 'online'|'offline'

Environment variables:
- MQTT_HOST (required)
- MQTT_PORT (required)
- DAA_HEALTH_INTERVAL_SEC (optional; default: 2 seconds)

This plugin starts automatically when imported.
"""
from __future__ import annotations
import os
import json
import socket
import threading
import time
from typing import Optional

import bluesky as bs
import paho.mqtt.client as mqtt
import logging
import logging.config

# Try to reuse repository logging config if present
try:
    with open('../logging_c2c.json', 'r') as f:
        config = json.load(f)
    logging.config.dictConfig(config)
except Exception:
    pass

logger = logging.getLogger("health_monitor")

HEARTBEAT_INTERVAL = float(os.getenv("DAA_HEALTH_INTERVAL_SEC", "2"))


class MQTTHealthClient(mqtt.Client):
    def __init__(self, monitor: 'HealthMonitor'):
        super().__init__()
        self.monitor = monitor

    def on_connect(self, client, userdata, flags, rc):
        logger.info("Health MQTT connected with result: %s", mqtt.error_string(rc))
        # Announce online state (retained)
        topic = "bluesky/status"
        payload = {"status": "online", "ts": int(time.time()), "sim_time": 0.0, "sim_dt": 0.0, "sim_state": 0, "ntraf": 0, "pid": os.getpid()}
        self.publish(topic, payload=json.dumps(payload), qos=1, retain=True)

    def on_disconnect(self, client, userdata, rc):
        logger.warning("Health MQTT disconnected: rc=%s", rc)


class HealthMonitor:
    def __init__(self):
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.mqtt_client = MQTTHealthClient(self)

    def start(self):
        host = os.environ.get("MQTT_HOST")
        port = int(os.environ.get("MQTT_PORT", "1883"))
        if not host:
            logger.error("MQTT_HOST not set; health monitor disabled")
            return

        # Set Last Will: broker publishes 'offline' if we crash
        will_topic = "bluesky/status"
        will_payload = {"status": "offline", "ts": int(time.time()), "sim_time": 0.0, "sim_dt": 0.0, "sim_state": 0, "ntraf": 0, "pid": os.getpid()}
        self.mqtt_client.will_set(will_topic, payload=json.dumps(will_payload), qos=1, retain=True)

        try:
            self.mqtt_client.connect(host, port, 60)
        except Exception as e:
            logger.error("Failed to connect to MQTT broker: %s", e)
            return

        self.mqtt_client.loop_start()

        self._thread = threading.Thread(target=self._run, name="health_heartbeat", daemon=True)
        self._thread.start()
        logger.info("Health monitor started: interval=%.1fs", HEARTBEAT_INTERVAL)

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        try:
            # Publish 'offline' on graceful stop
            topic = "bluesky/status"
            payload = {"status": "offline", "ts": int(time.time()), "sim_time": 0.0, "sim_dt": 0.0, "sim_state": 0, "ntraf": 0, "pid": os.getpid()}
            self.mqtt_client.publish(topic, payload=json.dumps(payload), qos=1, retain=True)
        except Exception:
            pass
        self.mqtt_client.loop_stop()

    def _run(self):
        topic = "bluesky/status"
        last_log = 0.0
        while not self._stop.is_set():
            payload = self._build_payload()
            try:
                self.mqtt_client.publish(topic, payload=json.dumps(payload), qos=0, retain=False)
                if logger.isEnabledFor(logging.DEBUG):
                    logger.debug(
                        "Health heartbeat published: topic=%s ts=%d sim_time=%.2f ntraf=%d",
                        topic,
                        payload.get('ts', 0),
                        payload.get('sim_time', 0.0),
                        payload.get('ntraf', 0),
                    )
            except Exception as e:
                # Rate-limit error logs
                now = time.time()
                if now - last_log > 10:
                    logger.error("Failed to publish heartbeat: %s", e)
                    last_log = now
            self._stop.wait(HEARTBEAT_INTERVAL)

    def _build_payload(self) -> dict:
        # Safe getters to avoid attribute errors if sim/traf not ready
        ts = int(time.time())
        sim_time = float(getattr(getattr(bs, 'sim', None), 'simt', 0.0) or 0.0)
        sim_dt = float(getattr(getattr(bs, 'sim', None), 'simdt', 0.0) or 0.0)
        sim_state = int(getattr(getattr(bs, 'sim', None), 'state', 0) or 0)
        ntraf = int(getattr(getattr(bs, 'traf', None), 'ntraf', 0) or 0)
        return {
            'status': 'online',
            'ts': ts,
            'sim_time': sim_time,
            'sim_dt': sim_dt,
            'sim_state': sim_state,
            'ntraf': ntraf,
            'pid': os.getpid(),
        }


# Plugin wiring
_monitor = HealthMonitor()


def init_plugin():
    """BlueSky plugin entrypoint."""
    config = {
        'plugin_name': 'health_monitor',
        'plugin_type': 'sim'
    }
    logger.info("health_monitor plugin initialized")
    _monitor.start()
    return config


def stop_plugin():
    """Optional stop hook if BlueSky supports it."""
    _monitor.stop()
