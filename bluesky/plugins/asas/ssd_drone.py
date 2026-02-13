''' Conflict resolution based on the SSD algorithm described in: https://repository.tudelft.nl/islandora/object/uuid%3A4b92f6b0-dc40-4946-a1ae-7efd0df79401?collection=education '''
import bluesky as bs
from bluesky.core import Entity
import json
import time
import os
import paho.mqtt.client as mqtt
from bluesky.traffic.asas import ConflictResolution
from bluesky.tools import geo
from bluesky.tools import areafilter
from bluesky.tools.aero import nm, Rearth
from bluesky import core
import numpy as np
import sys
import bluesky.plugins.c2c.c2c_ownstate_receiver as ownstate_receiver
# Try to import pyclipper
try:
    import pyclipper
except ImportError:
    print("Could not import pyclipper, RESO SSD will not function")

import logging
import logging.config
class UTCFormatter(logging.Formatter):
    converter = time.gmtime

logging_config_path = '../logging_c2c.json'
if not os.path.exists(logging_config_path):
    logging_config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'logging_c2c.json')

if os.path.exists(logging_config_path):
    with open(logging_config_path, 'r') as f:
        config = json.load(f)
    logging.config.dictConfig(config)
else:
    logging.basicConfig(level=logging.INFO)

logger = logging.getLogger("SSD_drone")

# Import profiling utilities
try:
    from bluesky.plugins.asas import profiling_utils as prof
    _profiling_available = True
except ImportError:
    _profiling_available = False
    logger.warning("Profiling utilities not available")

if bs.settings.DAA_profiling:
    import cProfile
    from pstats import SortKey
    if _profiling_available:
        prof.enable_profiling()
        prof.enable_cprofile()

c2c_avoid_request_publisher_loop_flag = 1

# Dummy context manager for when profiling is disabled
class _DummyContext:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        pass

def init_plugin():
    ''' Initialize the SSD_DRONE plugin '''
    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SSD_DRONE',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    logger.info("SSD_DRONE plugin initialized")

    if getattr(bs.settings, 'DAA_profiling', False):
        logger.info("Extra DAA profiling enabled")
    else:
        logger.info("Extra DAA profiling disabled")

    if getattr(bs.settings, 'avoid_ownship_only', False):
        logger.info("Sending avoidance requests only to aircraft registered in the C2C...")
    else:
        logger.info("Sending avoidance requests to all air traffic...")

    logger.info("Conflict detection radius is set to %(radius)s [nm]", {'radius': str(bs.settings.DAA_radius)})

    return config

def get_signed_area_polygon(xs, ys):
    """Vectorized calculation of signed area of polygon."""
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    xs_next = np.roll(xs, -1)
    ys_next = np.roll(ys, -1)
    signed_area = np.sum((xs_next - xs) * (ys_next + ys))
    return signed_area

class ConflictResolutionTime(core.Entity):
    ''' Entity of time when sent conflict resolution. '''
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.cr_time = np.array([])

    def create(self, n=1):
        ''' This function gets called automatically when new aircraft are created. '''
        super().create(n)
        # After base creation we can change the values in our own states for the new aircraft
        self.cr_time[-n:] = 0.0

conflictresolutiontime = ConflictResolutionTime()

class ConflictEarlyWarning(core.Entity):
    ''' Entity tracking conflict detection publish timing. '''
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.last_publish_time = np.array([])  # Time of last conflict detection publish

    def create(self, n=1):
        ''' This function gets called automatically when new aircraft are created. '''
        super().create(n)
        self.last_publish_time[-n:] = 0.0

conflictearlywarning = ConflictEarlyWarning()

class MQTTAvoidRequestPublisher(mqtt.Client):
    def __init__(self, C2CAvoidRequestPublisher):
        super().__init__()
        self.C2CAvoidRequestPublisher = C2CAvoidRequestPublisher
        self._publish_cache = {}  # Cache to track what was published

    def run(self):
        # Make Traffic publisher MQTT client
        mqtt_host = os.environ.get("MQTT_HOST")
        mqtt_port = os.environ.get("MQTT_PORT")
        if not mqtt_host or not mqtt_port:
            logger.warning("MQTT_HOST/MQTT_PORT not set; avoid_request publisher not started")
            return
        self.connect(mqtt_host, int(mqtt_port), 60)
        self.loop_start()

        while c2c_avoid_request_publisher_loop_flag == 1:
            logger.debug("Waiting for Avoid Request Publisher MQTT client to connect...")
            time.sleep(0.1)

    def on_connect(self, mqttc, obj, flags, rc):
        global c2c_avoid_request_publisher_loop_flag
        c2c_avoid_request_publisher_loop_flag = 0
        logger.info("Avoid Request Publisher MQTT client connect with result code: %(code)s", {'code': mqtt.error_string(rc)})
        return

    def on_message(self, mqttc, obj, msg):
        return

    def on_publish(self, mqttc, obj, mid):
        if logger.isEnabledFor(logging.DEBUG):
            # Retrieve cached message info if available
            msg_info = self._publish_cache.pop(mid, None)
            if msg_info:
                logger.debug("Avoid request published: topic=%s ac_id=%s lat=%d lon=%d alt=%d mid=%d",
                           msg_info['topic'], msg_info['ac_id'], msg_info['lat'],
                           msg_info['lon'], msg_info['alt'], mid)
            else:
                logger.debug("Avoid Request Publisher MQTT client published message with mid: %d", mid)
        return

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        return

    def on_log(self, mqttc, obj, level, string):
        return
    
    def publish_conflict_detection(self, ac_id, traffic_ids, time_to_conflict):
        """Publish early conflict detection warning."""
        body = {
            'ac_id': ac_id,
            'timestamp': int(time.time()),
            'traffic_ids': traffic_ids,
            'time_to_conflict': time_to_conflict
        }
        
        logger.debug("Sending conflict_detection: %(body)s", {'body': json.dumps(body)})
        self.publish('daa/conflict_detection', payload=json.dumps(body))
    
    def stop(self):
        self.loop_stop()


class C2CAvoidRequestPublisher(Entity):
    def __init__(self):
        super().__init__()
        # Start mqtt client to read out control commands
        self.mqtt_client = MQTTAvoidRequestPublisher(self)
        if getattr(bs.settings, 'c2c_enable_mqtt', True):
            self.mqtt_client.run()
        else:
            logger.info("C2C MQTT disabled; avoid_request publisher not started")

if getattr(bs.settings, 'c2c_enable_mqtt', True):
    avoid_request_publisher = C2CAvoidRequestPublisher()
else:
    avoid_request_publisher = None

class SSD_Drone(ConflictResolution):
    def loaded_pyclipper():
        """ Return true if pyclipper is successfully loaded """
        return "pyclipper" in sys.modules

    def detect(asas, traf):
        """ Detect all current conflicts """

        # Check if ASAS is ON first!
        if not asas.swasas:
            return

        # Construct the SSD
        constructSSD(asas, traf)

    def update(self, conf, ownship, intruder):
        """Update conflict resolution and run early conflict detection independently."""
        # Run early conflict detection FIRST, before standard resolution
        self.detect_early_conflicts(conf, ownship)
        
        # Then run the standard resolution update
        super().update(conf, ownship, intruder)

    def detect_early_conflicts(self, conf, ownship):
        """Detect conflicts using longer lookahead time and publish at regular intervals."""
        # Use the longer conflict detection lookahead time
        cd_dtlookahead = np.full(ownship.ntraf, bs.settings.conflict_detection_dtlookahead)
        
        # Run conflict detection with early warning lookahead
        confpairs_early, lospairs_early, inconf_early, tcpamax_early, qdr_early, \
            dist_early, dcpa_early, tcpa_early, tLOS_early = \
                bs.traf.cd.detect(ownship, ownship, conf.rpz, conf.hpz, cd_dtlookahead)
        
        # Filter aircraft that should be processed
        c2c_ownship_ids = (set(ownstate_receiver.c2c_ownstate_receiver.ownstate_objects.keys()) 
                          if bs.settings.avoid_ownship_only else None)
        
        current_time = time.time()
        publish_interval = bs.settings.conflict_detection_publish_interval

        # Publish full conflict state at regular intervals (only when in conflict)
        for i in range(ownship.ntraf):
            if not self._should_process_aircraft(i, ownship, c2c_ownship_ids):
                continue
            
            # Only publish if ownship is in conflict
            if not inconf_early[i]:
                continue

            time_since_last_publish = current_time - conflictearlywarning.last_publish_time[i]
            
            if time_since_last_publish >= publish_interval:
                # Collect all current conflicts for this aircraft
                traffic_in_conflict = [pair[1] for pair in confpairs_early 
                                      if pair[0] == ownship.id[i]]
                
                # Get time to conflict for all conflicts
                ttc_list = [float(tcpa_early[j]) for j, pair in enumerate(confpairs_early)
                           if pair[0] == ownship.id[i]]
                
                # Publish full conflict state
                if avoid_request_publisher and avoid_request_publisher.mqtt_client:
                    avoid_request_publisher.mqtt_client.publish_conflict_detection(
                        ownship.id[i],
                        traffic_in_conflict,
                        ttc_list
                    )
                    
                    logger.debug("Published conflict detection for %(ac_id)s: %(n_conflicts)d conflicts",
                               {'ac_id': ownship.id[i], 'n_conflicts': len(traffic_in_conflict)})
                
                # Update last publish time
                conflictearlywarning.last_publish_time[i] = current_time


    def resolve(self, conf, ownship, intruder):
        
        if bs.settings.DAA_profiling and _profiling_available:
            prof.start_cprofile()

        # Initialize SSD variables with ntraf
        with prof.profile_section("SSD.initializeSSD") if _profiling_available else _DummyContext():
            self.initializeSSD(conf, ownship.ntraf)

        # Construct the SSD
        with prof.profile_section("SSD.constructSSD") if _profiling_available else _DummyContext():
            self.constructSSD(conf, ownship)

        # Get resolved speed-vector
        with prof.profile_section("SSD.calculate_resolution") if _profiling_available else _DummyContext():
            self.calculate_resolution(conf, ownship)

        # Now assign resolutions to variables in the ASAS class
        # Start with current states, need a copy, otherwise it changes traf!
        newtrack = np.copy(ownship.hdg)
        newgs = np.copy(ownship.gs)
        # Calculate new track and speed
        # No need to cap the speeds, since SSD implicitly caps
        new_trk = np.arctan2(conf.asase, conf.asasn) * 180 / np.pi
        new_gs = np.sqrt(conf.asase ** 2 + conf.asasn ** 2)

        # Sometimes an aircraft is in conflict, but no solutions could be found
        # In that case it is assigned 0 by ASAS, but needs to handled
        asas_cmd = np.logical_and(conf.inconf, new_gs > 0)

        # Assign new track and speed for those that are in conflict
        newtrack[asas_cmd] = new_trk[asas_cmd]
        newgs[asas_cmd] = new_gs[asas_cmd]
        # Not needed as it is a 2D-implementation...
        newvs = ownship.vs

        # Cap the velocity
        newgscapped = np.maximum(ownship.perf.vmin, np.minimum(ownship.perf.vmax, newgs))

        alt = ownship.selalt

        if bs.settings.DAA_profiling and _profiling_available:
            prof.stop_cprofile()
            # Print stats every 10 calls to avoid excessive output
            if hasattr(self, '_profile_call_count'):
                self._profile_call_count += 1
            else:
                self._profile_call_count = 1
            
            if self._profile_call_count % 10 == 0:
                logger.info("="*80)
                logger.info("PROFILING REPORT (after %d resolve calls)", self._profile_call_count)
                logger.info("="*80)
                
                # Get stats and log them
                stats = prof.get_stats(sort_by='total_time', top_n=15)
                if stats:
                    logger.info("%-50s %8s %10s %10s %10s %10s %10s", 
                               'Function', 'Count', 'Total(s)', 'Avg(ms)', 'Min(ms)', 'Max(ms)', 'P95(ms)')
                    logger.info("-"*100)
                    for stat in stats:
                        name = stat['name']
                        if len(name) > 48:
                            name = "..." + name[-45:]
                        logger.info("%-50s %8d %10.3f %10.3f %10.3f %10.3f %10.3f",
                                   name,
                                   stat['count'],
                                   stat['total_time'],
                                   stat['avg_time']*1000,
                                   stat['min_time']*1000,
                                   stat['max_time']*1000,
                                   stat.get('p95', 0)*1000)
                
                summary = prof.get_summary()
                logger.info("="*80)
                logger.info("Total profiled calls: %d", summary.get('total_calls', 0))
                logger.info("Total profiled time: %.3fs", summary.get('total_time', 0))
                logger.info("="*80)

        return newtrack, newgscapped, newvs, alt


    def initializeSSD(self, conf, ntraf):
        """ Initialize variables for SSD """
        # Need to do it here, since ASAS.reset doesn't know ntraf
        conf.FRV = [None] * ntraf
        conf.ARV = [None] * ntraf
        # For calculation purposes
        conf.ARV_calc = [None] * ntraf
        conf.inrange = [None] * ntraf
        # asas.inconf       = np.zeros(ntraf, dtype=bool)
        # Index 2 for sequential solutions (RS7, RS8)
        conf.ARV_calc2 = [None] * ntraf
        conf.inrange2 = [None] * ntraf
        conf.inconf2 = np.zeros(ntraf, dtype=bool)
        # Stores resolution vector, also used in visualization
        conf.asasn = np.zeros(ntraf, dtype=np.float32)
        conf.asase = np.zeros(ntraf, dtype=np.float32)
        # Area calculation
        conf.FRV_area = np.zeros(ntraf, dtype=np.float32)
        conf.ARV_area = np.zeros(ntraf, dtype=np.float32)
        conf.ap_free = np.ones(ntraf, dtype=bool)

        # asas is an object of the ASAS class defined in asas.py


    @prof.profile_function("SSD._get_ssd_parameters") if _profiling_available else lambda f: f
    def _get_ssd_parameters(self):
        """Get SSD algorithm parameters and constants."""
        return {
            'N_angle': 180,  # Number of points on circle (discretization)
            'hsep': bs.settings.asas_pzr * nm,  # Horizontal separation [m]
            'margin': self.resofach,  # Safety margin for evasion
            'alpham': 0.4999 * np.pi,  # Maximum half-angle for VO [rad]
            'betalos': np.pi / 4,  # Minimum divertion angle for LOS [rad]
            'adsbmax': bs.settings.DAA_radius * nm,  # Maximum ADS-B range [m]
            'delay': bs.settings.system_delay  # Assumed delay of the entire system. Drone state is extrapolated into the future [s]
        }

    @prof.profile_function("SSD._compute_predicted_positions") if _profiling_available else lambda f: f
    def _compute_predicted_positions(self, ownship, delay):
        """Compute aircraft positions after delay period."""
        lat = ownship.lat + np.degrees(delay * ownship.gsnorth / Rearth)
        lon = ownship.lon + np.degrees(delay * ownship.gseast / ownship.coslat / Rearth)
        return lat, lon

    @prof.profile_function("SSD._create_velocity_circle") if _profiling_available else lambda f: f
    def _create_velocity_circle(self, N_angle):
        """Create unit circle for velocity obstacle construction."""
        angles = np.arange(0, 2 * np.pi, 2 * np.pi / N_angle)
        xyc = np.transpose(np.reshape(np.concatenate((np.sin(angles), np.cos(angles))), (2, N_angle)))
        return xyc

    @prof.profile_function("SSD._compute_pairwise_geometry") if _profiling_available else lambda f: f
    def _compute_pairwise_geometry(self, lat, lon, ntraf):
        """Compute bearing and distance between all aircraft pairs."""
        ind1, ind2 = self.qdrdist_matrix_indices(ntraf)
        qdr, dist = geo.qdrdist_matrix(lat[ind1], lon[ind1], lat[ind2], lon[ind2])
        
        qdr = np.asarray(qdr).reshape(np.shape(ind1))
        dist = np.asarray(dist).reshape(np.shape(ind1))
        qdr = np.deg2rad(qdr)
        dist = dist * nm
        
        return ind1, ind2, qdr, dist

    @prof.profile_function("SSD._compute_velocity_obstacle_vertices") if _profiling_available else lambda f: f
    def _compute_velocity_obstacle_vertices(self, hsepm, dist, qdr, alpham):
        """Calculate velocity obstacle vertices in relative velocity space."""
        # Prevent VO issues in LoS by clamping minimum distance
        dist = np.where(dist < hsepm, hsepm, dist)
        
        # Half-angle of velocity obstacle with safety margin
        alpha = np.arcsin(hsepm / dist)
        alpha = np.where(alpha > alpham, alpham, alpha)
        
        # Precompute trigonometric values
        sinqdr = np.sin(qdr)
        cosqdr = np.cos(qdr)
        tanalpha = np.tan(alpha)
        cosqdrtanalpha = cosqdr * tanalpha
        sinqdrtanalpha = sinqdr * tanalpha
        
        return {
            'sinqdr': sinqdr,
            'cosqdr': cosqdr,
            'cosqdrtanalpha': cosqdrtanalpha,
            'sinqdrtanalpha': sinqdrtanalpha
        }

    @prof.profile_function("SSD._should_process_aircraft") if _profiling_available else lambda f: f
    def _should_process_aircraft(self, i, ownship, c2c_ownship_ids):
        """Check if aircraft should be processed for SSD construction."""
        # Filter by C2C ownship registration if required
        if bs.settings.avoid_ownship_only and bs.traf.id[i] not in c2c_ownship_ids:
            return False
        
        # Check if geofence is defined and aircraft is within geofence
        geofence_defined, ownship_in_geofence, _ = self._check_geofence_status(ownship, i)
        
        # If geofence is not defined or ownship is outside geofence, skip SSD construction for this aircraft
        if not geofence_defined or not ownship_in_geofence:
            return False
        
        return True

    @prof.profile_function("SSD._get_velocity_limits") if _profiling_available else lambda f: f
    def _get_velocity_limits(self, ownship, i):
        """Get min/max velocity for aircraft, with validation."""
        vmin = ownship.perf.vmin[i]
        vmax = ownship.perf.vmax[i]
        
        # Check if performance data is available
        if vmin == vmax == 0:
            return None, None
        
        # Ensure minimum velocity is positive
        if vmin < 0.001:
            vmin = 0.001
        
        return vmin, vmax

    @prof.profile_function("SSD._create_velocity_circles") if _profiling_available else lambda f: f
    def _create_velocity_circles(self, xyc, vmin, vmax):
        """Create inner and outer velocity circles for SSD."""
        circle_tup = (
            tuple(map(tuple, np.flipud(xyc * vmax))),  # Outer circle CCW
            tuple(map(tuple, xyc * vmin))              # Inner circle CW
        )
        circle_lst = [
            list(map(list, np.flipud(xyc * vmax))),
            list(map(list, xyc * vmin))
        ]
        return circle_tup, circle_lst

    @prof.profile_function("SSD._set_no_conflict_ssd") if _profiling_available else lambda f: f
    def _set_no_conflict_ssd(self, i, circle_lst, vmin, vmax, FRV_loc, ARV_loc, ARV_calc_loc, 
                            FRV_area_loc, ARV_area_loc):
        """Set SSD values for aircraft with no nearby conflicts."""
        ARV_loc[i] = circle_lst
        FRV_loc[i] = []
        ARV_calc_loc[i] = ARV_loc[i]
        FRV_area_loc[i] = 0
        ARV_area_loc[i] = np.pi * (vmax ** 2 - vmin ** 2)

    @prof.profile_function("SSD._filter_nearby_aircraft") if _profiling_available else lambda f: f
    def _filter_nearby_aircraft(self, i, ind, dist, adsbmax, ntraf):
        """Get indices of aircraft within range."""
        i_other = np.delete(np.arange(0, ntraf), i)
        ac_adsb = np.where(dist[ind] < adsbmax)[0]
        ind = ind[ac_adsb]
        i_other = i_other[ac_adsb]
        
        # Mirror correction for velocity obstacles
        fix = np.ones(np.shape(i_other))
        fix[i_other < i] = -1
        
        return i_other, ind, fix

    @prof.profile_function("SSD._construct_velocity_obstacle_vertices") if _profiling_available else lambda f: f
    def _construct_velocity_obstacle_vertices(self, i_other, gseast, gsnorth, ind, fix, vmax, vo_trig):
        """Build velocity obstacle triangle vertices for each intruder."""
        x1 = (vo_trig['sinqdr'] + vo_trig['cosqdrtanalpha']) * 2 * vmax
        x2 = (vo_trig['sinqdr'] - vo_trig['cosqdrtanalpha']) * 2 * vmax
        y1 = (vo_trig['cosqdr'] - vo_trig['sinqdrtanalpha']) * 2 * vmax
        y2 = (vo_trig['cosqdr'] + vo_trig['sinqdrtanalpha']) * 2 * vmax
        
        x = np.concatenate((gseast[i_other],
                           x1[ind] * fix + gseast[i_other],
                           x2[ind] * fix + gseast[i_other]))
        y = np.concatenate((gsnorth[i_other],
                           y1[ind] * fix + gsnorth[i_other],
                           y2[ind] * fix + gsnorth[i_other]))
        
        x = np.transpose(x.reshape(3, np.shape(i_other)[0]))
        y = np.transpose(y.reshape(3, np.shape(i_other)[0]))
        xy = np.dstack((x, y))
        
        return xy

    @prof.profile_function("SSD._load_geofence_data") if _profiling_available else lambda f: f
    def _load_geofence_data(self, ownship, i):
        """Load and process geofence coordinates for aircraft."""
        try:
            geofence = areafilter.basic_shapes['GF_' + str(ownship.id[i])]
            coordinates = np.reshape(geofence.coordinates, 
                                    (int(len(geofence.coordinates) / 2), 2))
            
            lats_gf = coordinates[:, 0]
            lons_gf = coordinates[:, 1]
            qdrs_gf, dists_gf = geo.qdrdist(
                np.full_like(lats_gf, ownship.lat[i]),
                np.full_like(lons_gf, ownship.lon[i]),
                lats_gf,
                lons_gf
            )
            
            dists_gf = dists_gf * nm
            qdrs_gf_rad = np.deg2rad(qdrs_gf)
            xs_gf = dists_gf * np.sin(qdrs_gf_rad)
            ys_gf = dists_gf * np.cos(qdrs_gf_rad)
            
            # Ensure counter-clockwise order
            if get_signed_area_polygon(xs_gf, ys_gf) > 0:
                xs_gf = xs_gf[::-1]
                ys_gf = ys_gf[::-1]
            
            return xs_gf, ys_gf
        except:
            return None, None

    @prof.profile_function("SSD._compute_geofence_segments") if _profiling_available else lambda f: f
    def _compute_geofence_segments(self, xs_gf, ys_gf):
        """Compute geofence segment vectors and rotation matrices."""
        xs_gf_next = np.roll(xs_gf, -1)
        ys_gf_next = np.roll(ys_gf, -1)
        dxs_gf = xs_gf_next - xs_gf
        dys_gf = ys_gf_next - ys_gf
        
        phis_gf = np.arctan2(dys_gf, dxs_gf)
        cos_phis_gf = np.cos(phis_gf)
        sin_phis_gf = np.sin(phis_gf)
        x_hats_prime = np.transpose(np.array([cos_phis_gf, sin_phis_gf]))
        y_hats_prime = np.transpose(np.array([-sin_phis_gf, cos_phis_gf]))
        
        return phis_gf, x_hats_prime, y_hats_prime, xs_gf, ys_gf

    @prof.profile_function("SSD._add_intruder_vo_to_clipper") if _profiling_available else lambda f: f
    def _add_intruder_vo_to_clipper(self, pc, xy, j, dist, ind, hsepm, i_other, qdr, beta, vmax, i):
        """Add velocity obstacle for a single intruder to clipper."""
        if dist[ind[j]] > hsepm:
            # Normal triangular VO
            VO = pyclipper.scale_to_clipper(tuple(map(tuple, xy[j, :, :])))
        else:
            # Line-of-sight: use dart-tip shape
            qdr_los = qdr[ind[j]] + np.pi if i_other[j] < i else qdr[ind[j]]
            leg = 1.1 * vmax / np.cos(beta) * np.array([1, 1, 1, 0])
            angles_los = np.array([qdr_los + 2 * beta, qdr_los, qdr_los - 2 * beta, 0.])
            x_los = leg * np.sin(angles_los)
            y_los = leg * np.cos(angles_los)
            xy_los = np.vstack((x_los, y_los)).T
            VO = pyclipper.scale_to_clipper(tuple(map(tuple, xy_los)))
        
        pc.AddPath(VO, pyclipper.PT_CLIP, True)

    @prof.profile_function("SSD._add_geofence_vos_to_clipper") if _profiling_available else lambda f: f
    def _add_geofence_vos_to_clipper(self, pc, ownship, i, i_other, j, xs_gf, ys_gf, 
                                     phis_gf, x_hats_prime, y_hats_prime, N_angle, vmax):
        """Add geofence-based velocity obstacles for an intruder."""
        qdr_int, dist_int = geo.qdrdist(ownship.lat[i], ownship.lon[i], 
                                       ownship.lat[i_other[j]], ownship.lon[i_other[j]])
        qdr_int_rad = np.deg2rad(qdr_int)
        x_int = dist_int * nm * np.sin(qdr_int_rad)
        y_int = dist_int * nm * np.cos(qdr_int_rad)
        d_int = np.array([x_int, y_int])
        
        trk_int = np.deg2rad(ownship.trk[i_other[j]])
        gs_int = ownship.gs[i_other[j]]
        v_int = np.array([gs_int * np.sin(trk_int), gs_int * np.cos(trk_int)])
        
        # Find candidate geofence segments
        v_int_dot_y_hats_prime = y_hats_prime @ v_int
        candidate_gf_segments = np.where(v_int_dot_y_hats_prime < 0)[0]
        
        if len(candidate_gf_segments) == 0:
            return
        
        # Compute geometry for candidate segments
        d_int_dot_x_hats_prime = x_hats_prime[candidate_gf_segments] @ d_int
        d_int_dot_y_hats_prime = y_hats_prime[candidate_gf_segments] @ d_int
        ds_geo = -np.sum(((np.array([xs_gf[candidate_gf_segments], 
                                     ys_gf[candidate_gf_segments]])).T * 
                         y_hats_prime[candidate_gf_segments]), 1)
        
        phis_prime_gf = 0.5 * np.arctan2(-d_int_dot_x_hats_prime, d_int_dot_y_hats_prime)
        phis_total_gf = phis_gf[candidate_gf_segments] + phis_prime_gf
        
        # Rotation matrices for secondary axis system
        cos_phis_total = np.cos(phis_total_gf)
        sin_phis_total = np.sin(phis_total_gf)
        x_hats_2prime = np.transpose(np.array([cos_phis_total, sin_phis_total]))
        y_hats_2prime = np.transpose(np.array([-sin_phis_total, cos_phis_total]))
        
        # Dot products for VO geometry
        d_int_dot_x_hats_2prime = x_hats_2prime @ d_int
        d_int_dot_y_hats_2prime = y_hats_2prime @ d_int
        v_int_dot_x_hats_2prime = x_hats_2prime @ v_int
        v_int_dot_y_hats_2prime = y_hats_2prime @ v_int
        d_int_dot_v_int = np.dot(d_int, v_int) * np.ones(np.shape(d_int_dot_x_hats_2prime))
        
        # Geometric constants
        C1s = 1. + np.sin(phis_prime_gf) * d_int_dot_x_hats_2prime / ds_geo
        C2s = 1. + np.cos(phis_prime_gf) * d_int_dot_y_hats_2prime / ds_geo
        C3s = -2. * v_int_dot_x_hats_2prime - np.sin(phis_prime_gf) * d_int_dot_v_int / ds_geo
        C4s = -2. * v_int_dot_y_hats_2prime - np.cos(phis_prime_gf) * d_int_dot_v_int / ds_geo
        
        Cxs_2prime = -C3s / (2. * C1s)
        Cys_2prime = -C4s / (2. * C2s)
        
        # Semi-major axes for ellipse/hyperbola
        a2s = (-gs_int**2 + C2s * Cys_2prime**2) / C1s + Cxs_2prime**2
        b2s = (-gs_int**2 + C1s * Cxs_2prime**2) / C2s + Cys_2prime**2
        
        # Construct and add VOs
        for k in range(len(a2s)):
            if a2s[k] <= 0:
                continue
            
            if b2s[k] > 0:
                # Ellipse case
                ellipse_angles = np.linspace(0., 2. * np.pi, N_angle)
                rotated_xs = np.sqrt(a2s[k]) * np.cos(ellipse_angles) + Cxs_2prime[k]
                rotated_ys = np.sqrt(b2s[k]) * np.sin(ellipse_angles) + Cys_2prime[k]
            else:
                # Hyperbola case
                tmax = np.log((20. * vmax + np.sqrt(20.**2 * vmax**2 + a2s[k])) / np.sqrt(a2s[k]))
                tmin = -tmax
                if phis_prime_gf[k] > 0:
                    t = np.linspace(tmin, tmax, N_angle)
                    rotated_xs = -np.sqrt(a2s[k]) * np.cosh(t) + Cxs_2prime[k]
                else:
                    t = np.linspace(tmax, tmin, N_angle)
                    rotated_xs = np.sqrt(a2s[k]) * np.cosh(t) + Cxs_2prime[k]
                rotated_ys = np.sqrt(-b2s[k]) * np.sinh(t) + Cys_2prime[k]
            
            # Rotate back to original frame
            non_rotated_xs = (rotated_xs * np.cos(phis_total_gf[k]) - 
                            rotated_ys * np.sin(phis_total_gf[k]))
            non_rotated_ys = (rotated_xs * np.sin(phis_total_gf[k]) + 
                            rotated_ys * np.cos(phis_total_gf[k]))
            
            xy_gf = np.transpose(np.array([non_rotated_xs, non_rotated_ys]))
            xy_gf_tuple = tuple(map(tuple, xy_gf))
            
            try:
                VO = pyclipper.scale_to_clipper(xy_gf_tuple)
                pc.AddPath(VO, pyclipper.PT_CLIP, True)
            except:
                pass

    @prof.profile_function("SSD._finalize_ssd_regions") if _profiling_available else lambda f: f
    def _finalize_ssd_regions(self, ARV, FRV, circle_lst, vmin, vmax, ownship, i, xyc):
        """ Compute a smaller subset of the ARV around the current speed if possible."""
        if len(ARV) == 0:
            return [], circle_lst, [], np.pi * (vmax ** 2 - vmin ** 2), 0
        elif len(FRV) == 0:
            return circle_lst, [], circle_lst, 0, np.pi * (vmax ** 2 - vmin ** 2)
        
        # Normalize to list format
        if not isinstance(FRV[0][0], list):
            FRV = [FRV]
        if not isinstance(ARV[0][0], list):
            ARV = [ARV]
        
        FRV_area = self.area(FRV)
        ARV_area = self.area(ARV)
        
        # Compute smaller ARV ring around current speed
        pc2 = pyclipper.Pyclipper()

        # Pruning ring around current speed (clamped to [vmin, vmax])
        arv_speed_buffer = bs.settings.ARV_speed_buffer  # [m/s]
        v_outer = min(vmax, ownship.tas[i] + arv_speed_buffer)
        v_inner = max(vmin, ownship.tas[i] - arv_speed_buffer)
        xyp = (tuple(map(tuple, np.flipud(xyc * v_outer))),
               tuple(map(tuple, xyc * v_inner)))

        # Intersect ARV with the pruning ring (ARV as subject, ring as clip)
        pc2.AddPaths(pyclipper.scale_to_clipper(ARV), pyclipper.PT_SUBJECT, True)
        pc2.AddPaths(pyclipper.scale_to_clipper(xyp), pyclipper.PT_CLIP, True)

        ARV_calc = pyclipper.scale_from_clipper(
            pc2.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO))
        
        # Fallback to full ARV if no intersection close to current speed
        if len(ARV_calc) == 0:
            ARV_calc = ARV  # Use full ARV
    
        return ARV, FRV, ARV_calc, FRV_area, ARV_area

    @prof.profile_function("SSD.constructSSD") if _profiling_available else lambda f: f
    def constructSSD(self, conf, ownship):
        """
        Construct the State Space Diagram (SSD) for all aircraft.
        
        Computes the Free Reachable Velocity (FRV) and Allowable Resolution Velocity (ARV)
        regions by constructing velocity obstacles from nearby traffic and geofence constraints.
        
        Args:
            conf: Conflict resolution configuration object
            ownship: Traffic object containing all aircraft states
        """
        ntraf = ownship.ntraf
        if ntraf == 0:
            return
        
        # Get algorithm parameters
        params = self._get_ssd_parameters()
        N_angle = params['N_angle']
        hsep = params['hsep']
        hsepm = hsep * params['margin']
        alpham = params['alpham']
        betalos = params['betalos']
        adsbmax = params['adsbmax']
        delay = params['delay']
        beta = np.pi / 4 + betalos / 2
        
        # Get traffic data
        gsnorth = ownship.gsnorth
        gseast = ownship.gseast
        
        # Compute predicted positions after delay
        lat, lon = self._compute_predicted_positions(ownship, delay)
        
        # Create velocity circle template
        xyc = self._create_velocity_circle(N_angle)
        
        # Initialize result arrays
        FRV_loc = [None] * ntraf
        ARV_loc = [None] * ntraf
        ARV_calc_loc = [None] * ntraf
        FRV_area_loc = np.zeros(ntraf, dtype=np.float32)
        ARV_area_loc = np.zeros(ntraf, dtype=np.float32)
        
        # Compute pairwise geometry between all aircraft
        ind1, ind2, qdr, dist = self._compute_pairwise_geometry(lat, lon, ntraf)
        
        # Compute velocity obstacle vertices for all pairs
        vo_trig = self._compute_velocity_obstacle_vertices(hsepm, dist, qdr, alpham)
        
        # Precompute C2C ownship set for filtering
        c2c_ownship_ids = (set(ownstate_receiver.c2c_ownstate_receiver.ownstate_objects.keys()) 
                          if bs.settings.avoid_ownship_only else None)
        
        # Process each aircraft
        for i in range(ntraf):
            # Check if this aircraft should be processed
            if not self._should_process_aircraft(i, ownship, c2c_ownship_ids):
                continue
            
            logger.debug("Constructing SSD for %(ownship)s...", {'ownship': str(ownship.id[i])})

            # Only calculate SSD for aircraft in conflict
            if not conf.inconf[i]:
                continue
            
            # Get velocity limits
            vmin, vmax = self._get_velocity_limits(ownship, i)
            if vmin is None:
                continue
            
            # Create velocity circles based on vmax and vmin for this aircraft
            circle_tup, circle_lst = self._create_velocity_circles(xyc, vmin, vmax)
            
            # Find indices of nearby aircraft
            ind = np.where(np.logical_or(ind1 == i, ind2 == i))[0]
            
            if len(ind) == 0:
                # No aircraft nearby: full ARV, empty FRV
                self._set_no_conflict_ssd(i, circle_lst, vmin, vmax, FRV_loc, ARV_loc, 
                                         ARV_calc_loc, FRV_area_loc, ARV_area_loc)
                continue
            
            # Filter for aircraft within range
            i_other, ind, fix = self._filter_nearby_aircraft(i, ind, dist, adsbmax, ntraf)
            conf.inrange[i] = i_other
            
            # Construct velocity obstacle vertices for intruders
            xy = self._construct_velocity_obstacle_vertices(i_other, gseast, gsnorth, ind, fix, vmax, vo_trig)
            
            # Initialize clipper for geometric operations
            pc = pyclipper.Pyclipper()
            pc.AddPaths(pyclipper.scale_to_clipper(circle_tup), pyclipper.PT_SUBJECT, True)
            
            # Load geofence
            xs_gf, ys_gf = self._load_geofence_data(ownship, i)
            geofence_data = None
            if xs_gf is not None:
                geofence_data = self._compute_geofence_segments(xs_gf, ys_gf)
            
            # Add velocity obstacles for each intruder
            for j in range(len(i_other)):
                self._add_intruder_vo_to_clipper(pc, xy, j, dist, ind, hsepm, i_other, qdr, beta, vmax, i)
                
                # Add geofence-based VOs if geofence is active
                if geofence_data is not None:
                    phis_gf, x_hats_prime, y_hats_prime, xs_gf, ys_gf = geofence_data
                    self._add_geofence_vos_to_clipper(pc, ownship, i, i_other, j, xs_gf, ys_gf,
                                                     phis_gf, x_hats_prime, y_hats_prime, N_angle, vmax)
            
            # Execute clipper to compute FRV and ARV
            FRV_raw = pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
            ARV_raw = pc.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)
            
            # Scale the results back to original coordinate system
            FRV = pyclipper.scale_from_clipper(FRV_raw)
            ARV = pyclipper.scale_from_clipper(ARV_raw)
            
            # Finalize regions and compute ARV subset
            ARV_loc[i], FRV_loc[i], ARV_calc_loc[i], FRV_area_loc[i], ARV_area_loc[i] = \
                self._finalize_ssd_regions(ARV, FRV, circle_lst, vmin, vmax, ownship, i, xyc)
        
        # Store results in conflict resolution object
        conf.FRV = FRV_loc
        conf.ARV = ARV_loc
        conf.ARV_calc = ARV_calc_loc
        conf.FRV_area = FRV_area_loc
        conf.ARV_area = ARV_area_loc


    @prof.profile_function("SSD._find_closest_arv_point") if _profiling_available else lambda f: f
    def _find_closest_arv_point(self, ARV_polygons, gseast, gsnorth):
        """Find closest point on ARV boundary to current velocity.
        
        Args:
            ARV_polygons: List of ARV polygon exteriors
            gseast: Current east velocity component [m/s]
            gsnorth: Current north velocity component [m/s]
            
        Returns:
            Tuple of (x, y) coordinates of closest point, or (0., 0.) if no ARV
        """
        if ARV_polygons is None or len(ARV_polygons) == 0:
            return 0., 0.
            
        # Loop through all exteriors and append. Afterwards concatenate
        p = []
        q = []
        for j in range(len(ARV_polygons)):
            p.append(np.array(ARV_polygons[j]))
            q.append(np.diff(np.row_stack((p[j], p[j][0])), axis=0))
        p = np.concatenate(p)
        q = np.concatenate(q)
        
        # Calculate squared distance between edges
        l2 = np.sum(q ** 2, axis=1)
        # Catch l2 == 0 (exception)
        same = l2 < 1e-8
        l2[same] = 1.
        
        # Calc t - parameter along each edge
        t = np.sum((np.array([gseast, gsnorth]) - p) * q, axis=1) / l2
        # t must be limited between 0 and 1
        t = np.clip(t, 0., 1.)
        t[same] = 0.
        
        # Calculate closest point to each edge
        x1 = p[:, 0] + t * q[:, 0]
        y1 = p[:, 1] + t * q[:, 1]
        
        # Get distance squared and sort
        d2 = (x1 - gseast) ** 2 + (y1 - gsnorth) ** 2
        ind = np.argsort(d2)
        
        return x1[ind[0]], y1[ind[0]]

    @prof.profile_function("SSD._calculate_resolution_waypoint") if _profiling_available else lambda f: f
    def _calculate_resolution_waypoint(self, ownship, i, asase, asasn, tcpamax):
        """Calculate lat/lon/alt waypoint from velocity resolution.
        
        Args:
            ownship: Aircraft object
            i: Aircraft index
            asase: East component of resolution velocity [m/s]
            asasn: North component of resolution velocity [m/s]
            tcpamax: Time to closest point of approach [s]
            
        Returns:
            Tuple of (tres, qdr_res, dist_res, lat_res, lon_res, alt_res, dx_n_res, dy_n_res)
        """
        tres = tcpamax
        dx_res = asase * tres
        dy_res = asasn * tres
        qdr_res = np.rad2deg(np.arctan2(dx_res, dy_res))
        dist_res = np.sqrt(dx_res**2 + dy_res**2) / nm
        lat_res, lon_res = geo.qdrpos(ownship.lat[i], ownship.lon[i], qdr_res, dist_res)
        alt_res = ownship.alt[i]  # [m]
        
        # Calculate normalized direction vector
        dx_n_res = dx_res / (dist_res * nm) if dist_res > 0 else 0.
        dy_n_res = dy_res / (dist_res * nm) if dist_res > 0 else 0.
        
        return tres, qdr_res, dist_res, lat_res, lon_res, alt_res, dx_n_res, dy_n_res

    @prof.profile_function("SSD._check_geofence_status") if _profiling_available else lambda f: f
    def _check_geofence_status(self, ownship, i, lat_res=None, lon_res=None):
        """Check if geofence is defined and validate ownship/resolution positions.
        
        Args:
            ownship: Aircraft object
            i: Aircraft index
            lat_res: Latitude of position to check [deg]
            lon_res: Longitude of position to check [deg]
            
        Returns:
            Tuple of (geofence_defined, ownship_in_geofence, solution_in_geofence)
        """
        geofence_defined = False
        ownship_in_geofence = False
        solution_in_geofence = False
        
        try:
            geofence = areafilter.basic_shapes['GF_' + str(ownship.id[i])]
            geofence_defined = True
            
            # Check if ownship is within geofence
            ownship_in_geofence = areafilter.checkInside('GF_' + str(ownship.id[i]), ownship.lat[i], ownship.lon[i], 0)
            
            # Check if resolution waypoint is within geofence
            solution_in_geofence = areafilter.checkInside('GF_' + str(ownship.id[i]), lat_res, lon_res, 0) if lat_res is not None and lon_res is not None else False
            
            if not ownship_in_geofence:
                logger.warning("Aircraft %(ac)s is outside geofence bounds", {'ac': ownship.id[i]})
        except:
            pass
        
        return geofence_defined, ownship_in_geofence, solution_in_geofence

    @prof.profile_function("SSD._adjust_resolution_for_geofence") if _profiling_available else lambda f: f
    def _adjust_resolution_for_geofence(self, ownship, i, qdr_res, dx_n_res, dy_n_res):
        """Adjust resolution to closest geofence segment when solution is outside.
        
        Args:
            ownship: Aircraft object
            i: Aircraft index
            qdr_res: Resolution bearing [deg]
            dx_n_res: Normalized east component of resolution
            dy_n_res: Normalized north component of resolution
            
        Returns:
            Tuple of (lat_res, lon_res) adjusted to geofence boundary
        """
        # Loop through geofence coordinates
        geofence = areafilter.basic_shapes['GF_' + str(ownship.id[i])]
        coordinates = np.reshape(geofence.coordinates, (int(len(geofence.coordinates) / 2), 2))
        
        # Vectorize geofence coordinate processing
        lats_gf = coordinates[:, 0]
        lons_gf = coordinates[:, 1]
        qdrs_gf, dists_gf = geo.qdrdist(
            np.full_like(lats_gf, ownship.lat[i]),
            np.full_like(lons_gf, ownship.lon[i]),
            lats_gf,
            lons_gf
        )
        dists_gf = dists_gf * nm
        
        xs_gf = dists_gf * np.sin(np.deg2rad(qdrs_gf))  # [m] East
        ys_gf = dists_gf * np.cos(np.deg2rad(qdrs_gf))  # [m] North
        
        if (get_signed_area_polygon(xs_gf, ys_gf) > 0):
            xs_gf = xs_gf[::-1]
            ys_gf = ys_gf[::-1]
        
        # Generate data for each geofence segment 0 to 1, 1 to 2, 2 to 3 ..... n to 0.
        xs_gf_next = np.roll(xs_gf, -1)
        ys_gf_next = np.roll(ys_gf, -1)
        dxs_gf = xs_gf_next - xs_gf
        dys_gf = ys_gf_next - ys_gf
        
        # Calculate values (phis) of rotation of geofence segments
        phis_gf = np.arctan2(dys_gf, dxs_gf)
        y_hats_prime = np.array([-np.sin(phis_gf), np.cos(phis_gf)])
        d_geo = -(xs_gf * y_hats_prime[0] + ys_gf * y_hats_prime[1])
        dist_gf_frac = -(-np.sin(phis_gf) * dx_n_res + np.cos(phis_gf) * dy_n_res)
        
        projected_distances = d_geo[dist_gf_frac > 0] * (1. / dist_gf_frac[dist_gf_frac > 0])
        
        # Recalculate resolution
        dist_res = min(projected_distances) / nm
        lat_res, lon_res = geo.qdrpos(ownship.lat[i], ownship.lon[i], qdr_res, dist_res)
        
        return lat_res, lon_res

    @prof.profile_function("SSD._publish_avoid_request") if _profiling_available else lambda f: f
    def _publish_avoid_request(self, ownship, i, lat_res, lon_res, alt_res, tres, dist_res):
        """Publish MQTT avoid request message with timeout check.
        
        Args:
            ownship: Aircraft object
            i: Aircraft index
            lat_res: Resolution latitude
            lon_res: Resolution longitude
            alt_res: Resolution altitude [m]
            tres: Time to resolution [s]
            dist_res: Distance to resolution [nm]
            
        Returns:
            True if message was published, False otherwise
        """
        # Check timeout for conflict resolution
        current_time = time.time()
        delta_cr_time = current_time - conflictresolutiontime.cr_time[i]
        
        if delta_cr_time <= bs.settings.avoidance_request_publish_interval:
            logger.debug("Waiting for previous avoidance request to timeout: %(delta_cr_time)f < %(timeout)f seconds", {'delta_cr_time': delta_cr_time, 'timeout': bs.settings.avoidance_request_publish_interval})
            return False
        
        conflictresolutiontime.cr_time[i] = current_time
        
        # Send resolution over MQTT
        body = {
            'ac_id': ownship.id[i],
            'timestamp': int(time.time()),
            'waypoint': {
                'lat': int(lat_res * 10**7),
                'lon': int(lon_res * 10**7),
                'alt': int(alt_res * 10**3)
            },
            'tres': int(time.time() + float(tres)),
            'vres': float(dist_res * nm / tres) if not np.isclose(tres, 0.0) else 0.0
        }
        
        if avoid_request_publisher is None:
            logger.debug("C2C MQTT disabled; skipping avoid_request publish")
            return False

        logger.debug("Sending avoid_request: %(body)s", {'body': json.dumps(body)})
        msg_info = avoid_request_publisher.mqtt_client.publish('daa/avoid_request', 
                                                               payload=json.dumps(body))
        
        # Cache message info for debug logging
        if logger.isEnabledFor(logging.DEBUG) and msg_info.rc == mqtt.MQTT_ERR_SUCCESS:
            avoid_request_publisher.mqtt_client._publish_cache[msg_info.mid] = {
                'topic': 'daa/avoid_request',
                'ac_id': body['ac_id'],
                'lat': body['waypoint']['lat'],
                'lon': body['waypoint']['lon'],
                'alt': body['waypoint']['alt']
            }
        
        return True

    @prof.profile_function("SSD.calculate_resolution") if _profiling_available else lambda f: f
    def calculate_resolution(self, conf, ownship):
        """ Calculates closest conflict-free point according to ruleset """
        # Variables
        ARV = conf.ARV_calc
        gsnorth = ownship.gsnorth
        gseast = ownship.gseast
        ntraf = ownship.ntraf
        # Pre-compute C2C ownship set once
        c2c_ownship_ids = set(ownstate_receiver.c2c_ownstate_receiver.ownstate_objects.keys()) if bs.settings.avoid_ownship_only else None

        # Loop through SSDs of all aircraft
        for i in range(ntraf):
            
            # Only do avoidances with aircraft that should be processed
            if not self._should_process_aircraft(i, ownship, c2c_ownship_ids):
                continue
            
            logger.debug("Checking %(ownship)s for conflicts...", {'ownship': str(ownship.id[i])})
            
            # Only those that are in conflict need to resolve
            if conf.inconf[i] and ARV[i] is not None and len(ARV[i]) > 0:
                logger.debug("%(ownship)s is in conflict, resolving...", 
                           {'ownship': str(bs.traf.id[i])})

                # Find closest point on ARV boundary
                conf.asase[i], conf.asasn[i] = self._find_closest_arv_point(
                    ARV[i], gseast[i], gsnorth[i])
            else:
                # Those not in conflict or with no solutions (full ARV)
                conf.asase[i] = 0.
                conf.asasn[i] = 0.

        # Loop through resolutions
        for i in range(ntraf):

            # Only do avoidances with aircraft that should be processed
            if not self._should_process_aircraft(i, ownship, c2c_ownship_ids):
                continue
                
            if (conf.asase[i] != 0. and conf.asasn[i] != 0.):
                # Calculate resolution waypoint
                tres, qdr_res, dist_res, lat_res, lon_res, alt_res, dx_n_res, dy_n_res = \
                    self._calculate_resolution_waypoint(ownship, i, conf.asase[i], 
                                                       conf.asasn[i], conf.tcpamax[i])

                # Check geofence status
                geofence_defined, ownship_in_geofence, solution_in_geofence = \
                    self._check_geofence_status(ownship, i, lat_res, lon_res)

                # Adjust resolution if outside geofence
                if geofence_defined and ownship_in_geofence and not solution_in_geofence:
                    lat_res, lon_res = self._adjust_resolution_for_geofence(
                        ownship, i, qdr_res, dx_n_res, dy_n_res)
                    dist_res = np.sqrt(dx_n_res**2 + dy_n_res**2) / nm
                    solution_in_geofence = True

                # Publish MQTT avoid request if timeout elapsed and solution is valid
                self._publish_avoid_request(ownship, i, lat_res, lon_res, alt_res, 
                                            tres, dist_res)

            # reset resolution as external parties have to respond to it
            conf.asase[i] = gseast[i]
            conf.asasn[i] = gsnorth[i]

    def area(self, vset):
        """ This function calculates the area of the set of FRV or ARV """
        # Initialize A as it could be calculated iteratively
        A = 0
        # Check multiple exteriors
        if type(vset[0][0]) == list:
            # Calc every exterior separately
            for i in range(len(vset)):
                A += pyclipper.scale_from_clipper(
                    pyclipper.scale_from_clipper(pyclipper.Area(pyclipper.scale_to_clipper(vset[i]))))
        else:
            # Single exterior
            A = pyclipper.scale_from_clipper(
                pyclipper.scale_from_clipper(pyclipper.Area(pyclipper.scale_to_clipper(vset))))
        return A


    def qdrdist_matrix_indices(self, ntraf):
        """ This function gives the indices that can be used in the lon/lat-vectors """
        # The indices will be n*(n-1)/2 long
        # Only works for n >= 2, which is logical...
        # This is faster than np.triu_indices :)
        tmp_range = np.arange(ntraf - 1, dtype=np.int32)
        ind1 = np.repeat(tmp_range, (tmp_range + 1)[::-1])
        ind2 = np.ones(ind1.shape[0], dtype=np.int32)
        inds = np.cumsum(tmp_range[1:][::-1] + 1)
        np.put(ind2, inds, np.arange(ntraf * -1 + 3, 1))
        ind2 = np.cumsum(ind2, out=ind2)
        return ind1, ind2


    def minTLOS(self, conf, ownship, i, i_other, x1, y1, x, y):
        """ This function calculates the aggregated TLOS for all resolution points """
        # Get speeds of other AC in range
        x_other = ownship.gseast[i_other]
        y_other = ownship.gsnorth[i_other]
        # Get relative bearing [deg] and distance [nm]
        qdr, dist = geo.qdrdist(ownship.lat[i], ownship.lon[i], ownship.lat[i_other], ownship.lon[i_other])
        # Convert to SI
        qdr = np.deg2rad(qdr)
        dist *= nm
        # For vectorization, store lengths as W and L
        W = np.shape(x)[0]
        L = np.shape(x_other)[0]
        # Relative speed-components
        du = np.dot(x_other.reshape((L, 1)), np.ones((1, W))) - np.dot(np.ones((L, 1)), x.reshape((1, W)))
        dv = np.dot(y_other.reshape((L, 1)), np.ones((1, W))) - np.dot(np.ones((L, 1)), y.reshape((1, W)))
        # Relative speed + zero check
        vrel2 = du * du + dv * dv
        vrel2 = np.where(np.abs(vrel2) < 1e-6, 1e-6, vrel2)  # limit lower absolute value
        # X and Y distance
        dx = np.dot(np.reshape(dist * np.sin(qdr), (L, 1)), np.ones((1, W)))
        dy = np.dot(np.reshape(dist * np.cos(qdr), (L, 1)), np.ones((1, W)))
        # Time to CPA
        tcpa = -(du * dx + dv * dy) / vrel2
        # CPA distance
        dcpa2 = np.square(np.dot(dist.reshape((L, 1)), np.ones((1, W)))) - np.square(tcpa) * vrel2
        # Calculate time to LOS
        R2 = conf.rpz * conf.rpz
        swhorconf = dcpa2 < R2
        dxinhor = np.sqrt(np.maximum(0, R2 - dcpa2))
        dtinhor = dxinhor / np.sqrt(vrel2)
        tinhor = np.where(swhorconf, tcpa - dtinhor, 0.)
        tinhor = np.where(tinhor > 0, tinhor, 1e6)
        # Get index of best solution
        idx = np.argmax(np.sum(tinhor, 0))

        return idx
