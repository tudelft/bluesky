#!/usr/bin/env python3
"""
Real-time SSD visualization monitor for DAA scenarios.

Connects to MQTT broker and visualizes SSD computation in real-time
as scenarios are published by mqtt_scenario_publisher.py.

Usage:
    # Terminal 1: Run scenario publisher
    python DAA_validation/mqtt_scenario_publisher.py
    
    # Terminal 2: Run SSD monitor (in parallel)
    python DAA_validation/ssd_live_monitor.py
"""

import sys
import os
import json
import time
import math
import threading
import numpy as np
import paho.mqtt.client as mqtt

# Add BlueSky to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

# Initialize BlueSky in detached sim mode for SSD computation
import bluesky as bsky
bsky.init(mode='sim', detached=True)

# Set required settings BEFORE importing any plugins that need them
bsky.settings.set_variable_defaults(
    DAA_profiling=False,
    avoid_ownship_only=False,
    avoidance_timeout=4.0,
    ARV_speed_buffer=1.0,
    DAA_radius=2.0,
    system_delay=5.0,
    asas_pzr=0.0269978402,
    c2c_enable_mqtt=False,  # Critical: disable MQTT to prevent connection attempts
)

# Now safe to import SSD_Drone
from bluesky.plugins.asas.ssd_drone import SSD_Drone


class SSD_Drone_NoGeofence(SSD_Drone):
    """SSD implementation that skips geofence filtering for visualization."""

    def _should_process_aircraft(self, i, ownship, c2c_ownship_ids):
        return True
from bluesky.tools import geo
from bluesky.tools.aero import nm


class SSDLiveMonitor:
    """Monitor MQTT traffic and visualize SSD in real-time."""
    
    def __init__(self, broker_host="localhost", broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client = mqtt.Client(client_id="ssd_live_monitor")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Aircraft state tracking (ac_id -> state dict)
        self.aircraft_states = {}
        self.state_lock = threading.Lock()
        
        # Visualization
        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.active = False
        self.ownship_id = None
        self.intruder_id = None
        self.scenario_ownship_id = None
        self.scenario_intruder_id = None
        
        # Update control
        self.last_update = 0
        self.update_interval = 0.5  # seconds
        
        # Track recommended resolution velocities from SSD
        self.last_resolution_ve = None  # East component [m/s]
        self.last_resolution_vn = None  # North component [m/s]
        
        # Debug: Store raw pyclipper output for visualization
        self.debug_pyclipper_frv = None
        self.debug_pyclipper_arv = None
        
        print("SSD Live Monitor initialized")
        print(f"Connecting to MQTT broker at {broker_host}:{broker_port}...")
    
    def on_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker."""
        if rc == 0:
            print(f"✓ Connected to MQTT broker")
            # Subscribe to aircraft state topics
            client.subscribe("daa/ownstate")
            client.subscribe("daa/traffic")
            client.subscribe("daa/scenario")
            print("✓ Subscribed to daa/ownstate, daa/traffic, and daa/scenario")
            print("\nWaiting for aircraft data...")
        else:
            print(f"✗ Connection failed with code {rc}")
    
    def on_message(self, client, userdata, msg):
        """Callback when message received."""
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == "daa/scenario":
                if payload.get("event") == "start":
                    self._handle_scenario_start(payload)
                return
            
            # Handle ownstate and traffic messages
            if msg.topic in ["daa/ownstate", "daa/traffic"]:
                ac_id = payload.get('ac_id')
                if not ac_id:
                    return
                
                # Check for delete marker
                if payload.get('delete'):
                    with self.state_lock:
                        if ac_id in self.aircraft_states:
                            del self.aircraft_states[ac_id]
                    return
                
                # Convert from DAA format to internal state
                with self.state_lock:
                    vn_new = payload['vn'] / 1000
                    ve_new = payload['ve'] / 1000
                    gs_new = math.sqrt(vn_new**2 + ve_new**2)
                    
                    # Log velocity updates
                    if ac_id in self.aircraft_states:
                        gs_old = self.aircraft_states[ac_id]['gs']
                        if abs(gs_new - gs_old) > 0.1:  # Only log if velocity changed significantly
                            print(f"  ↑ {ac_id} velocity update: {gs_old:.1f} -> {gs_new:.1f} m/s")
                    
                    self.aircraft_states[ac_id] = {
                        'lat': payload['lat'] / 1e7,  # degE7 to degrees
                        'lon': payload['lon'] / 1e7,
                        'alt': payload['alt'] / 1000,  # mE3 to meters
                        'vn': vn_new,  # m/sE3 to m/s
                        've': ve_new,
                        'vd': payload['vd'] / 1000,
                        'gs': gs_new,
                        'trk': math.degrees(math.atan2(ve_new, vn_new)) % 360,
                        'is_ownship': msg.topic == "daa/ownstate"
                    }
        except Exception as e:
            print(f"Error parsing message: {e}")

    def _handle_scenario_start(self, payload):
        """Handle scenario start event and refresh plot state."""
        scenario_name = payload.get("scenario", "Unnamed Scenario")
        self.scenario_ownship_id = payload.get("ownship_id")
        intruders = payload.get("intruder_ids") or []
        self.scenario_intruder_id = intruders[0] if intruders else None

        with self.state_lock:
            self.aircraft_states.clear()

        self.active = False
        self.ownship_id = None
        self.intruder_id = None

        self._reset_plot(scenario_name)
        print(f"\n▶ Scenario start: {scenario_name}")
        if self.scenario_ownship_id and self.scenario_intruder_id:
            print(f"  Using IDs: ownship={self.scenario_ownship_id}, intruder={self.scenario_intruder_id}")
    
    def connect(self):
        """Connect to MQTT broker."""
        try:
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            time.sleep(1)  # Wait for connection
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def setup_visualization(self):
        """Initialize matplotlib figure."""
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))
        self.fig.canvas.manager.set_window_title("SSD Live Monitor - Real-time Visualization")
        print("\n✓ Visualization window opened")
        self._reset_plot("Waiting for scenario...")

    def _reset_plot(self, title):
        """Clear all axes and show a scenario title."""
        if not self.fig or not self.ax1 or not self.ax2:
            return
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_title("Geographic View (Ownship-Centered)", fontsize=12, fontweight="bold")
        self.ax2.set_title("Velocity Space (SSD)", fontsize=12, fontweight="bold")
        self.fig.suptitle(title, fontsize=14, fontweight="bold")
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def update_visualization(self):
        """Update the SSD visualization with current aircraft states."""
        current_time = time.time()
        if current_time - self.last_update < self.update_interval:
            return
        self.last_update = current_time
        
        with self.state_lock:
            if len(self.aircraft_states) < 2:
                if self.active:
                    print("\n⏸ Less than 2 aircraft - pausing visualization")
                    self.active = False
                return
            
            # Get ownship and first intruder
            ac_list = list(self.aircraft_states.keys())
            
            # Determine ownship (prefer scenario-provided, else marked ownship, else first)
            ownship_id = None
            if self.scenario_ownship_id in self.aircraft_states:
                ownship_id = self.scenario_ownship_id
            else:
                for ac_id in ac_list:
                    if self.aircraft_states[ac_id].get('is_ownship', False):
                        ownship_id = ac_id
                        break
            if ownship_id is None:
                ownship_id = ac_list[0]
            
            # Get first intruder (prefer scenario-provided, else any aircraft that isn't ownship)
            intruder_id = None
            if self.scenario_intruder_id in self.aircraft_states and self.scenario_intruder_id != ownship_id:
                intruder_id = self.scenario_intruder_id
            else:
                for ac_id in ac_list:
                    if ac_id != ownship_id:
                        intruder_id = ac_id
                        break
            
            if intruder_id is None:
                return  # Can't visualize without intruder
            
            # Track if aircraft changed
            if not self.active or ownship_id != self.ownship_id or intruder_id != self.intruder_id:
                print(f"\n▶ Visualizing: {ownship_id} (ownship) vs {intruder_id} (intruder)")
                self.ownship_id = ownship_id
                self.intruder_id = intruder_id
                self.active = True
            
            # Create traffic and conflict objects (pass IDs in correct order)
            traf = self._create_simple_traffic([ownship_id, intruder_id])
            conf = self._create_simple_conflict(traf)
            
            # Compute SSD
            try:
                ssd = SSD_Drone_NoGeofence()
                ssd.resofach = 1.0
                
                # Override separation requirement for drone scenarios (50m instead of 5 nm)
                import bluesky as bs
                original_pzr = bs.settings.asas_pzr
                original_delay = bs.settings.system_delay
                bs.settings.asas_pzr = 50 / nm  # 50 meters in nm
                bs.settings.system_delay = 0.0  # No prediction delay for static testing
                
                ssd.initializeSSD(conf, traf.ntraf)
                
                # Monkey-patch to capture pyclipper output for debugging
                original_execute = ssd.__class__.__dict__.get('_execute_pyclipper', None)
                def capture_pyclipper(frv_raw, arv_raw):
                    self.debug_pyclipper_frv = frv_raw
                    self.debug_pyclipper_arv = arv_raw
                
                ssd.constructSSD(conf, traf)

                # (pruned ARV check done on demand; no persistent debug output)
                
                # Store recommended resolution velocities (asase/asasn are East/North components)
                self.last_resolution_ve = conf.asase[0] if hasattr(conf, 'asase') and len(conf.asase) > 0 else None
                self.last_resolution_vn = conf.asasn[0] if hasattr(conf, 'asasn') and len(conf.asasn) > 0 else None

                # Update plot (now indices are guaranteed: 0=ownship, 1=intruder)
                self._update_plot(traf, conf, ssd, 0, 1)
                
                # Restore original settings after plotting
                bs.settings.asas_pzr = original_pzr
                bs.settings.system_delay = original_delay
            except Exception as e:
                print(f"SSD computation error: {e}")
                import traceback
                traceback.print_exc()
    
    def _create_simple_traffic(self, ac_ids):
        """Create a simple traffic object from current aircraft states in specified order."""
        class SimpleTraffic:
            def __init__(self, states, ordered_ids):
                self.ntraf = len(ordered_ids)
                self.id = ordered_ids
                
                # Build arrays in the specified order
                self.lat = np.array([states[ac_id]['lat'] for ac_id in ordered_ids])
                self.lon = np.array([states[ac_id]['lon'] for ac_id in ordered_ids])
                self.alt = np.array([states[ac_id]['alt'] for ac_id in ordered_ids])
                self.gs = np.array([states[ac_id]['gs'] for ac_id in ordered_ids])
                self.hdg = np.array([states[ac_id]['trk'] for ac_id in ordered_ids])
                self.trk = np.array([states[ac_id]['trk'] for ac_id in ordered_ids])
                self.tas = self.gs.copy()
                self.vs = np.zeros(self.ntraf)
                
                self.gsnorth = np.array([states[ac_id]['vn'] for ac_id in ordered_ids])
                self.gseast = np.array([states[ac_id]['ve'] for ac_id in ordered_ids])
                self.coslat = np.cos(np.radians(self.lat))
                
                # Performance data (set dynamically from current speeds)
                class PerfData:
                    def __init__(self, n):
                        self.vmin = np.ones(n) * 5.0
                        self.vmax = np.ones(n) * 20.0
                self.perf = PerfData(self.ntraf)
        
        return SimpleTraffic(self.aircraft_states, ac_ids)
    
    def _create_simple_conflict(self, traf):
        """Create a simple conflict resolution object."""
        class SimpleConf:
            def __init__(self, ntraf):
                self.ntraf = ntraf
                self.inconf = np.ones(ntraf, dtype=bool)  # Force SSD computation
                self.inrange = [[] for _ in range(ntraf)]  # Required by SSD
                self.FRV = [None] * ntraf  # For visualization
                self.ARV_calc = [None] * ntraf  # Finalized/pruned ARV
                self.FRV_area = np.zeros(ntraf)
                self.ARV_area = np.zeros(ntraf)
                self.asase = np.zeros(ntraf)  # Resolution velocity (East) [m/s]
                self.asasn = np.zeros(ntraf)  # Resolution velocity (North) [m/s]
                self.tcpamax = np.full(ntraf, 300.0)  # Max TCPA [s]
        
        return SimpleConf(traf.ntraf)
    
    def _update_plot(self, traf, conf, ssd, idx_own, idx_int):
        """Update the matplotlib figure with current SSD data."""
        self.ax1.clear()
        self.ax2.clear()
        
        # ===== Geographic view (ownship-centered) =====
        own_lat, own_lon = traf.lat[idx_own], traf.lon[idx_own]
        int_lat, int_lon = traf.lat[idx_int], traf.lon[idx_int]
        
        # Relative position
        int_qdr, int_dist = geo.qdrdist(own_lat, own_lon, int_lat, int_lon)
        int_dist_m = int_dist * nm
        int_x = int_dist_m * np.sin(np.radians(int_qdr))
        int_y = int_dist_m * np.cos(np.radians(int_qdr))
        
        self.ax1.set_title("Geographic View (Ownship-Centered)", fontsize=12, fontweight="bold")
        self.ax1.plot(0, 0, "go", markersize=12, label=f"Ownship ({traf.id[idx_own]})", zorder=5)
        self.ax1.plot(int_x, int_y, "r*", markersize=15, label=f"Intruder ({traf.id[idx_int]})", zorder=5)
        
        # Velocity vectors
        scale = 30
        self.ax1.arrow(0, 0, traf.gseast[idx_own] * scale, traf.gsnorth[idx_own] * scale,
                      head_width=5, head_length=5, fc="green", ec="green", alpha=0.7, linewidth=2)
        self.ax1.arrow(int_x, int_y, traf.gseast[idx_int] * scale, traf.gsnorth[idx_int] * scale,
                      head_width=5, head_length=5, fc="red", ec="red", alpha=0.7, linewidth=2)
        
        # Separation circle
        sep_req = 50
        circle = Circle((0, 0), sep_req, fill=False, edgecolor='blue', 
                       linestyle='--', linewidth=2, label=f'Sep. Req. ({sep_req}m)')
        self.ax1.add_patch(circle)
        
        self.ax1.set_xlabel("East (m)", fontsize=11)
        self.ax1.set_ylabel("North (m)", fontsize=11)
        self.ax1.grid(True, alpha=0.3)
        
        # Set axis limits based on actual aircraft positions with margin
        # Allow different x and y extents for better visibility
        margin = 50
        x_extent = max(abs(int_x) + margin, 100)
        y_extent = max(abs(int_y) + margin, 100)
        
        self.ax1.set_xlim(-x_extent, x_extent)
        self.ax1.set_ylim(-y_extent, y_extent)
        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.legend(loc="upper right", fontsize=9)
        
        # ===== Velocity space =====
        self.ax2.set_title("Velocity Space (SSD)", fontsize=12, fontweight="bold")
        
        # Use static velocity bounds for circles
        vmin_val = 5.0  # Static minimum velocity (m/s)
        vmax_val = 20.0  # Static maximum velocity (m/s)
        
        angles = np.linspace(0, 2 * np.pi, 180)
        self.ax2.plot(vmax_val * np.sin(angles), vmax_val * np.cos(angles), 
                     "b-", linewidth=2, label=f"Vmax ({vmax_val:.1f} m/s)", zorder=1)
        self.ax2.plot(vmin_val * np.sin(angles), vmin_val * np.cos(angles), 
                     "b--", linewidth=1, label=f"Vmin ({vmin_val:.1f} m/s)", zorder=1)
        
        # Plot FRV/ARV_calc regions FIRST (so they appear behind other elements)
        # Use ARV_calc (finalized/pruned version) for visualization
        frv_plotted = self._plot_regions(self.ax2, getattr(conf, "FRV", None), idx_own, 
                                         "orangered", "FRV (Forbidden)", alpha=0.4)
        arv_plotted = self._plot_regions(self.ax2, getattr(conf, "ARV_calc", None), idx_own, 
                                         "limegreen", "ARV (Allowable)", alpha=0.25)

        # NOTE: Removed manual VO cone plotting - it doesn't match what SSD actually uses
        # The SSD constructs VOs internally and clips them, which is shown in FRV region
        
        # Show debug info about regions
        if hasattr(conf, 'FRV') and conf.FRV and conf.FRV[idx_own]:
            frv_area = getattr(conf, 'FRV_area', [0])[idx_own] if hasattr(conf, 'FRV_area') else 0
            if frv_area > 0 and not frv_plotted:
                print(f"  ⚠ FRV area={frv_area:.1f} but plotting failed")
        if hasattr(conf, 'ARV') and conf.ARV and conf.ARV[idx_own]:
            arv_area = getattr(conf, 'ARV_area', [0])[idx_own] if hasattr(conf, 'ARV_area') else 0
            if arv_area > 0 and not arv_plotted:
                print(f"  ⚠ ARV area={arv_area:.1f} but plotting failed")
        
        # Current velocities (plot on top) - absolute velocity space
        self.ax2.plot(traf.gseast[idx_own], traf.gsnorth[idx_own], "go", 
                 markersize=10, label=f"Ownship V ({traf.gs[idx_own]:.1f} m/s)", zorder=6)
        self.ax2.plot(traf.gseast[idx_int], traf.gsnorth[idx_int], "r*", 
                 markersize=13, label=f"Intruder V ({traf.gs[idx_int]:.1f} m/s)", zorder=6)
        
        # Recommended avoidance velocity from SSD (if available and non-zero)
        if self.last_resolution_ve is not None and self.last_resolution_vn is not None:
            if self.last_resolution_ve != 0 or self.last_resolution_vn != 0:
                rec_gs = math.sqrt(self.last_resolution_ve**2 + self.last_resolution_vn**2)
                self.ax2.plot(self.last_resolution_ve, self.last_resolution_vn, "g*", 
                             markersize=20, label=f"Recommended V ({rec_gs:.1f} m/s)", zorder=7)
                # Draw arrow from current to recommended
                self.ax2.arrow(traf.gseast[idx_own], traf.gsnorth[idx_own],
                             self.last_resolution_ve - traf.gseast[idx_own],
                             self.last_resolution_vn - traf.gsnorth[idx_own],
                             head_width=0.3, head_length=0.3, fc="gold", ec="gold",
                             alpha=0.6, linewidth=2, zorder=6)
        
        self.ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        self.ax2.axvline(x=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        
        self.ax2.set_xlabel("East Velocity (m/s)", fontsize=11)
        self.ax2.set_ylabel("North Velocity (m/s)", fontsize=11)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_aspect('equal', adjustable='box')
        self.ax2.legend(loc="upper right", fontsize=9)
        
        max_v = max(vmax_val * 1.2, 25)
        self.ax2.set_xlim(-max_v, max_v)
        self.ax2.set_ylim(-max_v, max_v)
        
        # Update title
        conflict_status = "IN CONFLICT" if conf.inconf[idx_own] else "NO CONFLICT"
        self.fig.suptitle(f"Live SSD: {traf.id[idx_own]} vs {traf.id[idx_int]} - {conflict_status}", 
                         fontsize=14, fontweight="bold")
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def _plot_pyclipper_debug(self, ax, regions, label, color, linestyle='-', linewidth=1, from_clipper=False):
        """Plot raw pyclipper output for debugging (as outlines without fill).
        
        Args:
            ax: matplotlib axis
            regions: polygon regions (list of polygons, where each polygon is list of (x,y) points)
            label: label for the plot
            color: line color
            linestyle: line style ('-', '--', ':', etc.)
            linewidth: line width
            from_clipper: if True, scale results from pyclipper coordinates; else use as-is
        """
        if regions is None:
            return
        
        import pyclipper  # Import here to avoid issues if not available
        
        for poly in regions:
            if not poly:
                continue
            arr = np.asarray(poly)
            if arr.ndim != 2 or arr.shape[1] < 2:
                continue
            
            # If data comes from pyclipper, scale it back to original coordinates
            if from_clipper:
                arr = pyclipper.scale_from_clipper(arr.tolist() if isinstance(arr, np.ndarray) else arr)
                arr = np.asarray(arr)
            
            # Close the polygon by adding first point at end
            if len(arr) > 0:
                arr_closed = np.vstack([arr, arr[0]])
                ax.plot(arr_closed[:, 0], arr_closed[:, 1], color=color, 
                       linestyle=linestyle, linewidth=linewidth, alpha=0.7, label=label, zorder=2)

    def _plot_vo_cone(self, ax, traf, idx_own, idx_int, hsepm, alpham, vmax, label, color):
        """Plot a single VO cone for the intruder in absolute velocity space."""
        try:
            # Get intruder's velocity vector (ground speed in velocity space)
            gx = traf.gseast[idx_int]
            gy = traf.gsnorth[idx_int]
            
            # Compute bearing and distance in POSITION space to get VO geometry
            own_lat, own_lon = traf.lat[idx_own], traf.lon[idx_own]
            int_lat, int_lon = traf.lat[idx_int], traf.lon[idx_int]
            qdr_deg, dist_nm = geo.qdrdist(own_lat, own_lon, int_lat, int_lon)
            dist_m = max(dist_nm * nm, hsepm)

            qdr = math.radians(qdr_deg)
            alpha = math.asin(hsepm / dist_m)
            alpha = min(alpha, alpham)

            sinqdr = math.sin(qdr)
            cosqdr = math.cos(qdr)
            tanalpha = math.tan(alpha)

            # Scale cone edges - use 2*vmax as per SSD formulation
            # This ensures the cone extends far enough to show collision velocities
            x1 = (sinqdr + cosqdr * tanalpha) * 2 * vmax
            x2 = (sinqdr - cosqdr * tanalpha) * 2 * vmax
            y1 = (cosqdr - sinqdr * tanalpha) * 2 * vmax
            y2 = (cosqdr + sinqdr * tanalpha) * 2 * vmax

            # Cone should open in direction of intruder (away from apex)
            # The edges should point from intruder velocity toward collision velocities

            # VO cone vertices in velocity space (absolute velocities)
            # Apex at intruder velocity, edges opening toward potential collision velocities
            cone = np.array([
                [gx, gy],                           # VO apex at intruder's velocity
                [gx + x1, gy + y1],                 # Left edge (toward intruder direction)
                [gx + x2, gy + y2],                 # Right edge (toward intruder direction)
            ])

            # Plot as filled triangle in velocity space
            patch = Polygon(cone, closed=True, fill=True, alpha=0.2, 
                          edgecolor=color, facecolor=color, linewidth=2.0,
                          label=label, zorder=4)
            ax.add_patch(patch)
        except Exception as e:
            print(f"Error plotting VO cone: {e}")
            return
    
    @staticmethod
    def _point_in_polygon(point, polygon):
        """Check if a point is inside a polygon using ray casting."""
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside
    
    @staticmethod
    def _plot_regions(ax, regions, idx, color, label, alpha=0.3):
        """Plot SSD regions (FRV/ARV).
        
        Returns:
            bool: True if any polygons were successfully plotted, False otherwise.
        """
        if regions is None or idx >= len(regions):
            return False
        
        polys = regions[idx]
        if not polys:
            return False
        
        if polys and not isinstance(polys[0][0], (list, tuple, np.ndarray)):
            polys = [polys]
        
        plotted = False
        first = True
        for poly in polys:
            if not poly:
                continue
            arr = np.asarray(poly)
            if arr.ndim != 2 or arr.shape[1] < 2:
                continue
            patch = Polygon(arr, closed=True, fill=True, alpha=alpha, 
                          edgecolor=color, facecolor=color, linewidth=2.5,
                          label=label if first else None, zorder=3)
            ax.add_patch(patch)
            plotted = True
            first = False
        
        return plotted
    
    def visualize_static(self, ownship_state, intruder_state, ownship_id="OWN", intruder_id="INT"):
        """Visualize a static test case without MQTT connection.
        
        Args:
            ownship_state: dict with keys: lat, lon, alt, vn, ve, vd (velocities in m/s)
            intruder_state: dict with keys: lat, lon, alt, vn, ve, vd
            ownship_id: identifier for ownship
            intruder_id: identifier for intruder
        """
        # Populate aircraft states manually
        with self.state_lock:
            self.aircraft_states.clear()
            
            # Process ownship
            vn_own = ownship_state['vn']
            ve_own = ownship_state['ve']
            gs_own = math.sqrt(vn_own**2 + ve_own**2)
            self.aircraft_states[ownship_id] = {
                'lat': ownship_state['lat'],
                'lon': ownship_state['lon'],
                'alt': ownship_state['alt'],
                'vn': vn_own,
                've': ve_own,
                'vd': ownship_state.get('vd', 0),
                'gs': gs_own,
                'trk': math.degrees(math.atan2(ve_own, vn_own)) % 360,
                'is_ownship': True
            }
            
            # Process intruder
            vn_int = intruder_state['vn']
            ve_int = intruder_state['ve']
            gs_int = math.sqrt(vn_int**2 + ve_int**2)
            self.aircraft_states[intruder_id] = {
                'lat': intruder_state['lat'],
                'lon': intruder_state['lon'],
                'alt': intruder_state['alt'],
                'vn': vn_int,
                've': ve_int,
                'vd': intruder_state.get('vd', 0),
                'gs': gs_int,
                'trk': math.degrees(math.atan2(ve_int, vn_int)) % 360,
                'is_ownship': False
            }
        
        # Set up visualization (non-interactive for static plots)
        plt.ioff()  # Turn off interactive mode for static visualization
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 6))
        self.fig.canvas.manager.set_window_title("SSD Live Monitor - Static Visualization")
        print("\n✓ Visualization window opened")
        
        self.ownship_id = ownship_id
        self.intruder_id = intruder_id
        self.active = True
        
        # Force one update
        self.last_update = 0
        self.update_visualization()
        
        print("\n" + "="*80)
        print(f"Static visualization: {ownship_id} vs {intruder_id}")
        print("Close the window to exit")
        print("="*80 + "\n")
        
        # Keep window open (blocking call)
        plt.show(block=True)
    
    def run(self):
        """Main monitoring loop."""
        if not self.connect():
            print("\n✗ Could not connect to MQTT broker")
            print("Make sure the broker is running and accessible")
            return 1
        
        self.setup_visualization()
        
        print("\n" + "="*80)
        print("SSD Live Monitor - Running")
        print("="*80)
        print("Monitoring MQTT topics: daa/ownstate, daa/traffic")
        print("Press Ctrl+C to stop")
        print("="*80 + "\n")
        
        try:
            while True:
                self.update_visualization()
                time.sleep(0.1)  # Small sleep to prevent CPU spin
                
        except KeyboardInterrupt:
            print("\n\n✗ Stopped by user")
        finally:
            self.client.loop_stop()
            self.client.disconnect()
            plt.close('all')
            print("✓ Disconnected")
        
        return 0


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Real-time SSD visualization monitor for DAA scenarios",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  # Terminal 1: Start scenario publisher
  python DAA_validation/mqtt_scenario_publisher.py
  
  # Terminal 2: Start SSD monitor (run in parallel)
  python DAA_validation/ssd_live_monitor.py
        """
    )
    parser.add_argument("--host", default="localhost", help="MQTT broker host (default: localhost)")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port (default: 1883)")
    
    args = parser.parse_args()
    
    monitor = SSDLiveMonitor(args.host, args.port)
    return monitor.run()


if __name__ == "__main__":
    sys.exit(main())
