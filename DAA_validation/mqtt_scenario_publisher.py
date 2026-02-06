#!/usr/bin/env python3
"""
MQTT Scenario Publisher for BlueSky DAA API

Publishes validation test scenarios to BlueSky Docker container via MQTT.
Uses BlueSky's built-in autopilot for position/velocity simulation.
Only sets initial position, heading, and speed commands.

Usage:
    1. Start BlueSky Docker container with C2C plugins:
       docker run -d --name bluesky -p 1883:1883 -p 50000:50000 tudelftcnsatm/bluesky:latest
    2. Run validation scenarios:
       python mqtt_scenario_publisher.py
    3. Run specific scenario:
       python mqtt_scenario_publisher.py --scenario 1
"""

import json
import time
import math
from datetime import datetime
from typing import Dict, Any, List
import paho.mqtt.client as mqtt
from daa_validation_scenarios import get_all_scenarios, get_scenario_by_number, get_scenario_summary


def convert_to_daa_format(scenario: Dict[str, Any]) -> Dict[str, Any]:
    """
    Convert test scenario to DAA API format for initial state publishing.
    
    DAA API uses:
    - lat/lon: degE7 (degrees * 10^7)
    - alt: mE3 (meters * 10^3)
    - vn/ve/vd: m/sE3 (m/s * 10^3)
    
    All aircraft are published as ownstate so either can detect and avoid conflicts.
    BlueSky's autopilot will handle position/velocity updates after initial setup.
    """
    daa_messages = {
        "ownstate": [],
        "traffic": [],
        "geofence": None
    }
    
    for ac in scenario['aircraft']:
        # Convert velocity from ground speed and track to vn/ve components
        trk_rad = math.radians(ac['trk'])
        gs_ms = ac['gs']  # Already in m/s
        
        vn = int(gs_ms * math.cos(trk_rad) * 1000)  # North velocity [m/sE3]
        ve = int(gs_ms * math.sin(trk_rad) * 1000)  # East velocity [m/sE3]
        vd = int(-ac['vs'] * 1000)  # Down velocity [m/sE3] (negative of vertical speed)
        
        flight_state = {
            "ac_id": ac['id'],
            "priority": 1,  # Default priority
            "lat": int(ac['lat'] * 1e7),  # degE7
            "lon": int(ac['lon'] * 1e7),  # degE7
            "alt": int(ac['alt'] * 1000),  # mE3
            "vn": vn,
            "ve": ve,
            "vd": vd
        }
        
        # Publish all aircraft as ownstate so either can detect conflicts
        daa_messages["ownstate"].append(flight_state)
    
    # Convert geofence if present
    if scenario.get('geofence'):
        gf = scenario['geofence']
        geozone_points = []
        for lat, lon in gf['vertices']:
            geozone_points.append({
                "lat": int(lat * 1e7),  # degE7
                "lon": int(lon * 1e7)   # degE7
            })
        
        daa_messages["geofence"] = {
            "ac_id": gf['ac_id'],
            "geozone": geozone_points
        }
    
    return daa_messages


def create_test_scenario_1_headon():
    """DEPRECATED - Use daa_validation_scenarios.py instead"""
    from daa_validation_scenarios import create_scenario_1_head_on_basic
    return create_scenario_1_head_on_basic()


def create_test_scenario_2_crossing():
    """DEPRECATED - Use daa_validation_scenarios.py instead"""
    from daa_validation_scenarios import create_scenario_3_crossing_90deg
    return create_scenario_3_crossing_90deg()


def create_test_scenario_3_overtaking():
    """DEPRECATED - Use daa_validation_scenarios.py instead"""
    from daa_validation_scenarios import create_scenario_5_overtaking
    return create_scenario_5_overtaking()


class ScenarioPublisher:
    """MQTT publisher for DAA test scenarios"""
    
    def __init__(self, broker_host="localhost", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = mqtt.Client(client_id="scenario_publisher")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.received_status = []
        self.received_avoid = {}  # Dict: ac_id -> latest avoid request
        self.connected = False
        self.aircraft_states = {}  # Track current aircraft states for following waypoints

    def cleanup_aircraft(self, ac_ids: List[str]):
        """Remove aircraft from DAA/traffic feeds to avoid cross-scenario bleed."""
        if not ac_ids:
            return
        # Single-pass best-effort delete markers with last-known state
        try:
            for ac_id in ac_ids:
                ac = self.aircraft_states.get(ac_id, {
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt": 0.0,
                    "trk": 0.0,
                    "gs": 0.0,
                })
                hdg_rad = math.radians(ac.get("trk", 0.0))
                spd = ac.get("gs", 0.0)
                vn = spd * math.cos(hdg_rad)
                ve = spd * math.sin(hdg_rad)
                vd = 0.0

                msg = {
                    "ac_id": ac_id,
                    "lat": int(ac.get("lat", 0.0) * 1e7),
                    "lon": int(ac.get("lon", 0.0) * 1e7),
                    "alt": int(ac.get("alt", 0.0) * 1000),
                    "vn": int(vn * 1000),
                    "ve": int(ve * 1000),
                    "vd": int(vd * 1000),
                    "delete": True,
                }
                # QoS 0 to minimize broker load and avoid crashes
                self.client.publish("daa/ownstate", json.dumps(msg), qos=0)
                self.client.publish("daa/traffic", json.dumps(msg), qos=0)
        except Exception as exc:
            print(f"  Cleanup skipped due to error: {exc}")
    
    def on_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            print(f"✓ Connected to MQTT broker at {self.broker_host}:{self.broker_port}")
            self.connected = True
            # Subscribe to response topics
            client.subscribe("daa/status")
            client.subscribe("daa/avoid_request")
            client.subscribe("bluesky/status")
            client.subscribe("bluesky/state")  # Subscribe to BlueSky state updates
        else:
            print(f"✗ Connection failed with code {rc}")
    
    def on_message(self, client, userdata, msg):
        """Callback when message received"""
        try:
            if msg.topic == "bluesky/status":
                pass  # Silently ignore status updates to reduce spam
            elif msg.topic == "daa/status":
                payload = json.loads(msg.payload.decode())
                self.received_status.append(payload)
                print(f"  DAA Status: inconf={payload.get('inconf')}, inreso={payload.get('inreso')}")
            elif msg.topic == "daa/avoid_request":
                payload = json.loads(msg.payload.decode())
                ac_id = payload.get('ac_id', 'unknown')
                self.received_avoid[ac_id] = payload
                wp = payload.get('waypoint', {})
                print(f"  ✓ Avoid Request for {ac_id}:")
                print(f"    Waypoint: lat={wp.get('lat')/1e7:.6f}, lon={wp.get('lon')/1e7:.6f}, alt={wp.get('alt')/1000:.1f}m")
                print(f"    Speed: {wp.get('speed', 0):.2f} m/s")
            elif msg.topic == "bluesky/result":
                payload = json.loads(msg.payload.decode())
                if not payload.get('success'):
                    print(f"  ✗ Command failed: {payload.get('message', 'Unknown error')}")
        except Exception as e:
            print(f"  Error parsing message: {e}")
    
    def connect(self):
        """Connect to MQTT broker"""
        try:
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            # Wait for connection
            timeout = 5
            start = time.time()
            while not self.connected and (time.time() - start) < timeout:
                time.sleep(0.1)
            return self.connected
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def update_aircraft_toward_waypoint(self, ac_id: str, avoid_request: Dict[str, Any], 
                                        dt: float) -> float:
        """
        Update aircraft heading toward avoidance waypoint.
                
        Args:
            ac_id: Aircraft ID
            avoid_request: Avoid request with waypoint
            dt: Time step (seconds)
        
        Returns:
            New heading command (degrees)
        """
        if ac_id not in self.aircraft_states:
            return 0.0
        
        ac = self.aircraft_states[ac_id]
        wp = avoid_request.get('waypoint', {})
        
        # Convert waypoint from DAA format
        target_lat = wp.get('lat', 0) / 1e7
        target_lon = wp.get('lon', 0) / 1e7
        
        # Calculate direction to waypoint
        lat_diff = target_lat - ac['lat']
        lon_diff = target_lon - ac['lon']
        
        # Calculate distance (simplified, good for small distances)
        horiz_dist = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Convert degrees to meters
        
        # Only update if there's significant distance (>5m to avoid oscillation near waypoint)
        if horiz_dist > 5:
            # Calculate track toward waypoint
            trk_rad = math.atan2(lon_diff, lat_diff)
            desired_trk = math.degrees(trk_rad)
            
            # Normalize desired track to 0-360 range
            if desired_trk < 0:
                desired_trk += 360
            
            # Get current track
            current_trk = ac['trk']
            
            # Calculate shortest path to desired track
            heading_diff = desired_trk - current_trk
            
            # Normalize heading_diff to [-180, 180] range
            if heading_diff > 180:
                heading_diff -= 360
            elif heading_diff < -180:
                heading_diff += 360
            
            # Limit turn rate to max 45 degrees per iteration
            max_turn_per_iteration = 45.0
            
            if abs(heading_diff) > max_turn_per_iteration:
                if heading_diff > 0:
                    new_trk = current_trk + max_turn_per_iteration
                else:
                    new_trk = current_trk - max_turn_per_iteration
            else:
                new_trk = current_trk + heading_diff
            
            # Normalize final track to 0-360
            ac['trk'] = new_trk % 360
            return ac['trk']
        
        return ac['trk']
    
    def publish_scenario(self, scenario: Dict[str, Any], duration: float = 10.0, rate_hz: float = 1.0) -> List[str]:
        """
        Publish a scenario and simulate aircraft movement ourselves.
        
        We maintain aircraft positions and update them each iteration based on
        heading and velocity. When avoidance waypoints arrive, we change heading
        to point toward the waypoint and keep that heading persistent.
        
        Args:
            scenario: Test scenario dict
            duration: How long to publish messages (seconds)
            rate_hz: Publishing rate (Hz)
        """
        print(f"\n{'='*80}")
        print(f"Scenario: {scenario['name']}")
        print(f"{'='*80}")
        print(f"Description: {scenario.get('description', 'N/A')}")
        print(f"Expected: {scenario.get('expected', 'N/A')}")
        
        daa_msgs = convert_to_daa_format(scenario)
        
        # Show what we're publishing
        print(f"\nConfiguration:")
        ac_ids = [ac['id'] for ac in scenario['aircraft']]
        print(f"  Aircraft: {ac_ids}")
        if daa_msgs['geofence']:
            gf = daa_msgs['geofence']
            print(f"  Geofence: {len(gf['geozone'])} vertices for {gf['ac_id']}")
        
        # Initialize aircraft state tracking from original scenario
        self.aircraft_states = {}
        self.ownship_id = None  # Track the ownship (first aircraft)
        
        for idx, ac in enumerate(scenario['aircraft']):
            state = ac.copy()
            # Add home waypoint: extend current track far into distance (50 km ahead at current heading)
            hdg_rad = math.radians(ac['trk'])
            dist_m = 50000  # 50 km
            dlat = (dist_m * math.cos(hdg_rad)) / 111000
            dlon = (dist_m * math.sin(hdg_rad)) / 111000
            state['home_lat'] = ac['lat'] + dlat
            state['home_lon'] = ac['lon'] + dlon
            self.aircraft_states[ac['id']] = state
            
            # First aircraft is the ownship
            if idx == 0:
                self.ownship_id = ac['id']

        # Publish scenario start message for external plotters
        try:
            scenario_start = {
                "event": "start",
                "scenario": scenario.get("name", "Unnamed Scenario"),
                "aircraft_ids": [ac['id'] for ac in scenario['aircraft']],
                "ownship_id": self.ownship_id,
                "intruder_ids": [ac['id'] for ac in scenario['aircraft'] if ac['id'] != self.ownship_id],
                "timestamp": time.time(),
            }
            self.client.publish("daa/scenario", json.dumps(scenario_start), qos=0)
            print("✓ Published scenario start to daa/scenario")
        except Exception as exc:
            print(f"  Scenario start publish failed: {exc}")
        
        # Publish geofence once at the start
        if daa_msgs['geofence']:
            self.client.publish("daa/geofence", json.dumps(daa_msgs['geofence']))
            print(f"\n✓ Published geofence")
        
        # Clear previous responses
        self.received_status.clear()
        self.received_avoid.clear()
        
        # Publish states at specified rate
        interval = 1.0 / rate_hz
        iterations = int(duration * rate_hz)
        
        print(f"\nPublishing at {rate_hz} Hz for {duration}s...")
        print("-" * 80)
        
        # Track IDs for cleanup after scenario completes
        scenario_ac_ids = list(self.aircraft_states.keys())
        
        # Track which waypoints have been applied (ac_id -> waypoint tuple)
        applied_waypoints = {}

        for i in range(iterations):
            timestamp = time.time()
            
            # Check proximity to avoidance waypoints
            ac_ids_to_clear = []
            for ac_id in list(self.received_avoid.keys()):
                if ac_id in self.aircraft_states:
                    ac = self.aircraft_states[ac_id]
                    waypoint = self.received_avoid[ac_id].get('waypoint', {})
                    
                    # Convert waypoint from DAA format
                    target_lat = waypoint.get('lat', 0) / 1e7
                    target_lon = waypoint.get('lon', 0) / 1e7
                    
                    # Calculate distance to waypoint (simplified, good for small distances)
                    lat_diff = target_lat - ac['lat']
                    lon_diff = target_lon - ac['lon']
                    horiz_dist = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Convert degrees to meters
                    
                    # If within 25 meters of waypoint, mark as cleared
                    if horiz_dist < 25:  # 25 meters clearance threshold
                        print(f"    ✓ {ac_id} cleared avoidance waypoint (distance: {horiz_dist:.1f}m)")
                        ac_ids_to_clear.append(ac_id)
            
            # Remove cleared waypoints
            for ac_id in ac_ids_to_clear:
                del self.received_avoid[ac_id]
                # Reset applied waypoint so home waypoint can be engaged
                if ac_id in applied_waypoints:
                    del applied_waypoints[ac_id]
            
            # Update heading: toward avoidance waypoint if in conflict, else toward home waypoint
            for ac_id in self.aircraft_states:
                if ac_id in self.received_avoid:
                    # Aircraft has avoidance waypoint - head toward it
                    waypoint = self.received_avoid[ac_id].get('waypoint', {})
                    # Create a unique ID for this waypoint
                    waypoint_id = (waypoint.get('lat'), waypoint.get('lon'), waypoint.get('alt'))
                    
                    # Only apply waypoint if it's new (different from last applied)
                    if applied_waypoints.get(ac_id) != waypoint_id:
                        self.update_aircraft_toward_waypoint(ac_id, self.received_avoid[ac_id], interval)
                        applied_waypoints[ac_id] = waypoint_id
                        print(f"    Heading updated for {ac_id} toward avoidance waypoint")
                else:
                    # No avoidance waypoint - head toward home waypoint
                    ac = self.aircraft_states[ac_id]
                    home_waypoint = {
                        'waypoint': {
                            'lat': int(ac['home_lat'] * 1e7),
                            'lon': int(ac['home_lon'] * 1e7),
                            'alt': int(ac['alt'] * 1000),
                        }
                    }
                    # Create unique ID for home waypoint
                    home_id = ('home', ac['home_lat'], ac['home_lon'])
                    
                    # Update toward home waypoint (allow continuous heading adjustments)
                    # Only print status message if switching from avoidance to home
                    was_in_avoidance = applied_waypoints.get(ac_id) not in [None, home_id]
                    self.update_aircraft_toward_waypoint(ac_id, home_waypoint, interval)
                    applied_waypoints[ac_id] = home_id
                    if was_in_avoidance:
                        print(f"    {ac_id} resuming toward home waypoint")
            
            # Update aircraft positions based on current heading and speed
            for ac_id in self.aircraft_states:
                ac = self.aircraft_states[ac_id]
                
                # Convert heading to radians (0=north, 90=east, 180=south, 270=west)
                hdg_rad = math.radians(ac['trk'])
                
                # Calculate position change in this time step
                # Distance = speed * time
                distance_m = ac['gs'] * interval
                
                # Convert to lat/lon change
                # Approximate: 1 degree = 111,000 meters
                dlat = (distance_m * math.cos(hdg_rad)) / 111000
                dlon = (distance_m * math.sin(hdg_rad)) / 111000
                
                # Update position
                ac['lat'] += dlat
                ac['lon'] += dlon
            
            # Publish updated states to DAA system
            for ac_id in self.aircraft_states:
                ac = self.aircraft_states[ac_id]
                
                # Convert velocity components from heading and speed
                hdg_rad = math.radians(ac['trk'])
                spd = ac['gs']  # m/s
                
                # North/East/Down velocity components
                vn = spd * math.cos(hdg_rad)  # North component
                ve = spd * math.sin(hdg_rad)  # East component
                vd = 0  # No vertical velocity for now (level flight)
                
                # Create DAA format message
                daa_state = {
                    'ac_id': ac_id,
                    'lat': int(ac['lat'] * 1e7),      # Latitude in degrees * 1e7
                    'lon': int(ac['lon'] * 1e7),      # Longitude in degrees * 1e7
                    'alt': int(ac['alt'] * 1000),      # Altitude in meters * 1000
                    'vn': int(vn * 1000),              # North velocity in m/s * 1000
                    've': int(ve * 1000),              # East velocity in m/s * 1000
                    'vd': int(vd * 1000)               # Down velocity in m/s * 1000
                }
                
                # Publish to DAA system: ownship on daa/ownstate, intruders on daa/traffic
                if ac_id == self.ownship_id:
                    self.client.publish("daa/ownstate", json.dumps(daa_state))
                else:
                    self.client.publish("daa/traffic", json.dumps(daa_state))
            
            if i == 0 or (i + 1) % max(1, int(rate_hz)) == 0:
                print(f"  [{i+1}/{iterations}] Published state updates")
            
            # Wait for next iteration
            elapsed = time.time() - timestamp
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)
        
        print("-" * 80)
        print(f"✓ Completed '{scenario['name']}'")
        print(f"  Status messages: {len(self.received_status)}")
        print(f"  Avoid requests: {len(self.received_avoid)}")
        
        # Show conflict detection results
        if self.received_status:
            conflicts = sum(1 for s in self.received_status if s.get('inconf'))
            resolutions = sum(1 for s in self.received_status if s.get('inreso'))
            print(f"  Conflicts detected: {conflicts}/{len(self.received_status)}")
            print(f"  In resolution: {resolutions}/{len(self.received_status)}")
        
        if self.received_avoid:
            print(f"  Avoidance waypoints generated: {len(self.received_avoid)}")
            print(f"  Aircraft following waypoints: {', '.join(self.received_avoid.keys())}")
        
        # Clean up aircraft state for next scenario
        self.aircraft_states.clear()
        self.received_status.clear()
        self.received_avoid.clear()

        # Return IDs for upstream cleanup coordination
        return scenario_ac_ids
    
    def disconnect(self):
        """Disconnect from broker"""
        self.client.loop_stop()
        self.client.disconnect()
        print("\n✓ Disconnected from MQTT broker")


def main():
    """Main test runner"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Publish DAA validation scenarios via MQTT",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=get_scenario_summary()
    )
    parser.add_argument("--host", default="localhost", help="MQTT broker host (default: localhost)")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port (default: 1883)")
    parser.add_argument("--duration", type=float, default=40.0, help="Duration to publish each scenario (seconds)")
    parser.add_argument("--rate", type=float, default=2.0, help="Publishing rate (Hz)")
    parser.add_argument("--scenario", type=str, help="Run specific scenario (1-11) or 'all' for all scenarios")
    parser.add_argument("--list", action="store_true", help="List all available scenarios and exit")
    parser.add_argument("--pause", type=float, default=10.0, help="Pause between scenarios (seconds)")
    
    args = parser.parse_args()
    
    if args.list:
        print(get_scenario_summary())
        return 0
    
    print("="*80)
    print("BlueSky DAA Validation - MQTT Scenario Publisher")
    print("="*80)
    
    # Get scenarios to run
    if args.scenario:
        if args.scenario.lower() == 'all':
            scenarios = get_all_scenarios()
            print(f"\nRunning all {len(scenarios)} validation scenarios")
        else:
            try:
                scenario_num = int(args.scenario)
                scenarios = [get_scenario_by_number(scenario_num)]
                print(f"\nRunning scenario {scenario_num}")
            except ValueError:
                print(f"\n✗ Error: Invalid scenario '{args.scenario}'")
                print("\nUse --scenario <number> (1-11), --scenario all, or --list")
                return 1
    else:
        scenarios = get_all_scenarios()
        print(f"\nRunning all {len(scenarios)} validation scenarios")
    
    # Create publisher and connect
    publisher = ScenarioPublisher(args.host, args.port)
    
    if not publisher.connect():
        print("\n✗ Could not connect to MQTT broker at {}:{}".format(args.host, args.port))
        print("\nTroubleshooting:")
        print("  1. Ensure BlueSky Docker container is running with C2C plugins enabled")
        print("  2. Check MQTT port is exposed: -p 1883:1883")
        print("  3. Verify container uses settings_c2c.cfg for DAA functionality")
        return 1
    
    try:
        last_ac_ids: List[str] = []

        # Publish each scenario
        for i, scenario in enumerate(scenarios, 1):
            # Do not send delete markers; just wait between scenarios
            if last_ac_ids:
                print("\nWaiting extra pause to let prior scenario settle...")
                time.sleep(args.pause)

            last_ac_ids = publisher.publish_scenario(scenario, duration=args.duration, rate_hz=args.rate)
            
            # Pause between scenarios
            if i < len(scenarios):
                print(f"\n⏸  Waiting {args.pause}s before next scenario...")
                time.sleep(args.pause)
        
        print(f"\n{'='*80}")
        print(f"✓ All scenarios completed successfully!")
        print(f"{'='*80}")
        
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
    finally:
        publisher.disconnect()
    
    return 0


if __name__ == "__main__":
    exit(main())
