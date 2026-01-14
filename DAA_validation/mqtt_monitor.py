#!/usr/bin/env python3
"""
Quick diagnostic to monitor all MQTT traffic from BlueSky
"""

import paho.mqtt.client as mqtt
import json
import sys

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✓ Connected to MQTT broker")
        print("Subscribing to all topics (#)...\n")
        client.subscribe("#")
    else:
        print(f"✗ Connection failed with code {rc}")

def on_message(client, userdata, msg):
    topic = msg.topic
    try:
        # Try to parse as JSON
        payload = json.loads(msg.payload.decode())
        payload_str = json.dumps(payload, indent=2)
    except:
        # Plain text
        payload_str = msg.payload.decode()
    
    print(f"Topic: {topic}")
    print(f"Payload:\n{payload_str}")
    print("-" * 80)

def main():
    print("="*80)
    print("BlueSky MQTT Traffic Monitor")
    print("="*80)
    print("Listening to all MQTT topics...")
    print("Press Ctrl+C to stop\n")
    
    client = mqtt.Client(client_id="mqtt_monitor")
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect("localhost", 1883, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n\n✓ Stopped by user")
        client.disconnect()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
