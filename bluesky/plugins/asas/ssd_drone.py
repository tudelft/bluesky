''' Conflict resolution based on the SSD algorithm described in: https://repository.tudelft.nl/islandora/object/uuid%3A4b92f6b0-dc40-4946-a1ae-7efd0df79401?collection=education '''
import bluesky as bs
import json
import time
import os
import paho.mqtt.client as mqtt
from bluesky.traffic.asas import ConflictResolution
from bluesky.tools import geo
from bluesky.tools import areafilter
from bluesky.tools.aero import nm
from bluesky import core
import numpy as np
# Try to import pyclipper
try:
    import pyclipper
except ImportError:
    print("Could not import pyclipper, RESO SSD will not function")



# TODO: not completely migrated yet to class-based implementation

def init_plugin():

    # Addtional initilisation code

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'SSD_DRONE',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim'
    }

    return config

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
        self.cr_time[-n:] = [0 for _ in range(n)]

conflictresolutiontime = ConflictResolutionTime()

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


class SSD_Drone(ConflictResolution):
    def loaded_pyclipper():
        """ Return true if pyclipper is successfully loaded """
        import sys
        return "pyclipper" in sys.modules

    def detect(asas, traf):
        """ Detect all current conflicts """

        # Check if ASAS is ON first!
        if not asas.swasas:
            return

        # Construct the SSD
        constructSSD(asas, traf)


    def resolve(self, conf, ownship, intruder):
        # Initialize SSD variables with ntraf
        self.initializeSSD(conf, ownship.ntraf)

        # Construct the SSD
        self.constructSSD(conf, ownship)

        # Get resolved speed-vector
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


    def constructSSD(self, conf, ownship):
        """ Calculates the FRV and ARV of the SSD """
        N = 0
        # Parameters
        N_angle = 180  # [-] Number of points on circle (discretization)
        hsep = bs.settings.asas_pzr  # [m] Horizontal separation (5 NM)
        margin = self.resofach  # [-] Safety margin for evasion
        hsepm = hsep * margin  # [m] Horizontal separation with safety margin
        alpham = 0.4999 * np.pi  # [rad] Maximum half-angle for VO
        betalos = np.pi / 4  # [rad] Minimum divertion angle for LOS (45 deg seems optimal)
        adsbmax = 65. * nm  # [m] Maximum ADS-B range
        beta = np.pi / 4 + betalos / 2

        # Relevant info from traf
        gsnorth = ownship.gsnorth
        gseast = ownship.gseast
        lat = ownship.lat
        lon = ownship.lon
        ntraf = ownship.ntraf
        hdg = ownship.hdg
        gs_ap = ownship.ap.tas
        hdg_ap = ownship.ap.trk
        apnorth = np.cos(hdg_ap / 180 * np.pi) * gs_ap
        apeast = np.sin(hdg_ap / 180 * np.pi) * gs_ap

        # Local variables, will be put into asas later
        FRV_loc = [None] * ownship.ntraf
        ARV_loc = [None] * ownship.ntraf
        # For calculation purposes
        ARV_calc_loc = [None] * ownship.ntraf
        FRV_area_loc = np.zeros(ownship.ntraf, dtype=np.float32)
        ARV_area_loc = np.zeros(ownship.ntraf, dtype=np.float32)

        # # Use velocity limits for the ring-shaped part of the SSD
        # Discretize the circles using points on circle
        angles = np.arange(0, 2 * np.pi, 2 * np.pi / N_angle)
        # Put points of unit-circle in a (180x2)-array (CW)
        xyc = np.transpose(np.reshape(np.concatenate((np.sin(angles), np.cos(angles))), (2, N_angle)))

        # If no traffic
        if ntraf == 0:
            return

        # # If only one aircraft
        # elif ntraf == 1:
        #     # Map them into the format ARV wants. Outercircle CCW, innercircle CW
        #     ARV_loc[0] = circle_lst
        #     FRV_loc[0] = []
        #     ARV_calc_loc[0] = ARV_loc[0]
        #     # Calculate areas and store in asas
        #     FRV_area_loc[0] = 0
        #     ARV_area_loc[0] = np.pi * (vmax ** 2 - vmin ** 2)
        #     return

        # Function qdrdist_matrix needs 4 vectors as input (lat1,lon1,lat2,lon2)
        # To be efficient, calculate all qdr and dist in one function call
        # Example with ntraf = 5:   ind1 = [0,0,0,0,1,1,1,2,2,3]
        #                           ind2 = [1,2,3,4,2,3,4,3,4,4]
        # This way the qdrdist is only calculated once between every aircraft
        # To get all combinations, use this function to get the indices
        ind1, ind2 = self.qdrdist_matrix_indices(ntraf)
        # Get absolute bearing [deg] and distance [nm]
        # Not sure abs/rel, but qdr is defined from [-180,180] deg, w.r.t. North
        [qdr, dist] = geo.qdrdist_matrix(lat[ind1], lon[ind1], lat[ind2], lon[ind2])
        # Put result of function from matrix to ndarray
        qdr = np.reshape(np.array(qdr), np.shape(ind1))
        dist = np.reshape(np.array(dist), np.shape(ind1))
        # SI-units from [deg] to [rad]
        qdr = np.deg2rad(qdr)
        # Get distance from [nm] to [m]
        dist = dist * nm

        # In LoS the VO can't be defined, act as if dist is on edge
        dist[dist < hsepm] = hsepm

        # Calculate vertices of Velocity Obstacle (CCW)
        # These are still in relative velocity space, see derivation in appendix
        # Half-angle of the Velocity obstacle [rad]
        # Include safety margin
        alpha = np.arcsin(hsepm / dist)
        # Limit half-angle alpha to 89.982 deg. Ensures that VO can be constructed
        alpha[alpha > alpham] = alpham
        # Relevant sin/cos/tan
        sinqdr = np.sin(qdr)
        cosqdr = np.cos(qdr)
        tanalpha = np.tan(alpha)
        cosqdrtanalpha = cosqdr * tanalpha
        sinqdrtanalpha = sinqdr * tanalpha

        # Consider every aircraft
        for i in range(ntraf):
            # Calculate SSD only for aircraft in conflict (See formulas appendix)
            if conf.inconf[i]:

                vmin = ownship.perf.vmin[i]
                vmax = ownship.perf.vmax[i]

                # in the first time step, ASAS runs before perf, which means that his value will be zero
                # and the SSD cannot be constructed
                if vmin == vmax == 0:
                    continue

                if vmin < 0.001:
                    vmin = 0.001

                # Map them into the format pyclipper wants. Outercircle CCW, innercircle CW
                circle_tup = (tuple(map(tuple, np.flipud(xyc * vmax))), tuple(map(tuple, xyc * vmin)))
                circle_lst = [list(map(list, np.flipud(xyc * vmax))), list(map(list, xyc * vmin))]

                # Relevant x1,y1,x2,y2 (x0 and y0 are zero in relative velocity space)
                x1 = (sinqdr + cosqdrtanalpha) * 2 * vmax
                x2 = (sinqdr - cosqdrtanalpha) * 2 * vmax
                y1 = (cosqdr - sinqdrtanalpha) * 2 * vmax
                y2 = (cosqdr + sinqdrtanalpha) * 2 * vmax

                # SSD for aircraft i
                # Get indices that belong to aircraft i
                ind = np.where(np.logical_or(ind1 == i, ind2 == i))[0]
                # Check whether there are any aircraft in the vicinity
                if len(ind) == 0:
                    # No aircraft in the vicinity
                    # Map them into the format ARV wants. Outercircle CCW, innercircle CW
                    ARV_loc[i] = circle_lst
                    FRV_loc[i] = []
                    ARV_calc_loc[i] = ARV_loc[i]
                    # Calculate areas and store in asas
                    FRV_area_loc[i] = 0
                    ARV_area_loc[i] = np.pi * (vmax ** 2 - vmin ** 2)
                else:
                    # The i's of the other aircraft
                    i_other = np.delete(np.arange(0, ntraf), i)
                    # Aircraft that are within ADS-B range
                    ac_adsb = np.where(dist[ind] < adsbmax)[0]
                    # Now account for ADS-B range in indices of other aircraft (i_other)
                    ind = ind[ac_adsb]
                    i_other = i_other[ac_adsb]
                    conf.inrange[i] = i_other
                    # VO from 2 to 1 is mirror of 1 to 2. Only 1 to 2 can be constructed in
                    # this manner, so need a correction vector that will mirror the VO
                    fix = np.ones(np.shape(i_other))
                    fix[i_other < i] = -1

                    # Get vertices in an x- and y-array of size (ntraf-1)*3x1
                    x = np.concatenate((gseast[i_other],
                                        x1[ind] * fix + gseast[i_other],
                                        x2[ind] * fix + gseast[i_other]))
                    y = np.concatenate((gsnorth[i_other],
                                        y1[ind] * fix + gsnorth[i_other],
                                        y2[ind] * fix + gsnorth[i_other]))
                    # Reshape [(ntraf-1)x3] and put arrays in one array [(ntraf-1)x3x2]
                    x = np.transpose(x.reshape(3, np.shape(i_other)[0]))
                    y = np.transpose(y.reshape(3, np.shape(i_other)[0]))
                    xy = np.dstack((x, y))

                    # Make a clipper object
                    pc = pyclipper.Pyclipper()
                    # Add circles (ring-shape) to clipper as subject
                    pc.AddPaths(pyclipper.scale_to_clipper(circle_tup), pyclipper.PT_SUBJECT, True)

                    # If there is a geofence, calculate relative variables:
                    geofence_defined = False
                    try:
                        areafilter.basic_shapes['GF_' + str(ownship.id[i])]
                    except:
                        pass
                    else:
                        geofence_defined = True
                        # Load geofence shape
                        geofence = areafilter.basic_shapes['GF_' + str(ownship.id[i])]
                        # Loop through geofence coordinates
                        coordinates = np.reshape(geofence.coordinates, (int(len(geofence.coordinates) / 2), 2))
                        qdrs_gf = np.array([]) # [deg] in hdg CW
                        dists_gf = np.array([]) # [m]
                        for k in range(len(coordinates)):
                            # Calculate relative qdrs and distances of geofence points w.r.t. ownship
                            qdr_gf, dist_gf = geo.qdrdist(ownship.lat[i], ownship.lon[i], coordinates[k][0], coordinates[k][1])
                            qdrs_gf = np.append(qdrs_gf, qdr_gf)
                            dists_gf = np.append(dists_gf, dist_gf * nm)

                        qdrs_gf_rad = np.deg2rad(qdrs_gf)
                        xs_gf = dists_gf * np.sin(qdrs_gf_rad) # [m] East
                        ys_gf = dists_gf * np.cos(qdrs_gf_rad) # [m] North

                        
                        # Generate data for each geofence segment 0 to 1, 1 to 2, 2 to 3 ..... n to 0.
                        dxs_gf = np.array([])
                        dys_gf = np.array([])
                        for k in range(len(coordinates)):
                            x_from = xs_gf[k]
                            y_from = ys_gf[k]
                            # if last elament (needs to be connected to first element)
                            if k == (len(coordinates) - 1):
                                x_to = xs_gf[0]
                                y_to = ys_gf[0]
                            else:
                                x_to = xs_gf[k + 1]
                                y_to = ys_gf[k + 1]
                            
                            dxs_gf = np.append(dxs_gf, x_to - x_from)
                            dys_gf = np.append(dys_gf, y_to - y_from)

                        # calculate values (phis) of rotation of geofence segments
                        phis_gf = np.arctan2(dys_gf, dxs_gf)
                        x_hats_prime = np.transpose(np.array([np.cos(phis_gf), np.sin(phis_gf)]))
                        y_hats_prime = np.transpose(np.array([-np.sin(phis_gf), np.cos(phis_gf)]))

                    # Add each other other aircraft to clipper as clip
                    for j in range(np.shape(i_other)[0]):
                        ## Debug prints
                        ## print(traf.id[i] + " - " + traf.id[i_other[j]])
                        ## print(dist[ind[j]])
                        # Scale VO when not in LOS
                        if dist[ind[j]] > hsepm:
                            # Normally VO shall be added of this other a/c
                            VO = pyclipper.scale_to_clipper(tuple(map(tuple, xy[j, :, :])))
                        else:
                            # Pair is in LOS, instead of triangular VO, use darttip
                            # Check if bearing should be mirrored
                            if i_other[j] < i:
                                qdr_los = qdr[ind[j]] + np.pi
                            else:
                                qdr_los = qdr[ind[j]]
                            # Length of inner-leg of darttip
                            leg = 1.1 * vmax / np.cos(beta) * np.array([1, 1, 1, 0])
                            # Angles of darttip
                            angles_los = np.array([qdr_los + 2 * beta, qdr_los, qdr_los - 2 * beta, 0.])
                            # Calculate coordinates (CCW)
                            x_los = leg * np.sin(angles_los)
                            y_los = leg * np.cos(angles_los)
                            # Put in array of correct format
                            xy_los = np.vstack((x_los, y_los)).T
                            # Scale darttip
                            VO = pyclipper.scale_to_clipper(tuple(map(tuple, xy_los)))
                        # Add scaled VO to clipper
                        pc.AddPath(VO, pyclipper.PT_CLIP, True)

                        # Add Geofence if available
                        if (geofence_defined):
                            # Determine relative distance vector w.r.t. intruder
                            qdr_int, dist_int = geo.qdrdist(ownship.lat[i], ownship.lon[i], ownship.lat[i_other[j]], ownship.lon[i_other[j]])
                            qdr_int_rad = np.deg2rad(qdr_int)
                            x_int = dist_int * nm * np.sin(qdr_int_rad)
                            y_int = dist_int * nm * np.cos(qdr_int_rad)
                            d_int = np.array([x_int, y_int])
                            trk_int = np.deg2rad(ownship.trk[i_other[j]])
                            gs_int = ownship.gs[i_other[j]]
                            v_int = np.array([gs_int * np.sin(trk_int), gs_int * np.cos(trk_int)])

                            v_int_dot_y_hats_prime = np.array([])
                            for k in range(len(y_hats_prime)):
                                v_int_dot_y_hats_prime = np.append(v_int_dot_y_hats_prime, np.dot(v_int, y_hats_prime[k]))
                            candidate_gf_segments = np.where(v_int_dot_y_hats_prime < 0)[0]

                            d_int_dot_x_hats_prime = np.array([])
                            d_int_dot_y_hats_prime = np.array([])

                            ds_geo = np.array([]) # [m] Array of distanced w.r.t. geofence of wonship
                            for k in candidate_gf_segments:
                                d_int_dot_x_hats_prime = np.append(d_int_dot_x_hats_prime, np.dot(d_int, x_hats_prime[k]))
                                d_int_dot_y_hats_prime = np.append(d_int_dot_y_hats_prime, np.dot(d_int, y_hats_prime[k]))

                                ds_geo = np.append(ds_geo, -np.dot(np.array([xs_gf[k], ys_gf[k]]), y_hats_prime[k]))
                            
                            phis_prime_gf = 0.5 * np.arctan2(-1. * d_int_dot_x_hats_prime, d_int_dot_y_hats_prime)

                            # Total rotation angle
                            phis_total_gf = phis_gf[candidate_gf_segments] + phis_prime_gf

                            # Secondary axis system primary axes
                            x_hats_2prime = np.transpose(np.array([np.cos(phis_total_gf), np.sin(phis_total_gf)]))
                            y_hats_2prime = np.transpose(np.array([-np.sin(phis_total_gf), np.cos(phis_total_gf)]))
                            
                            # Arrays of dot products
                            d_int_dot_x_hats_2prime = np.array([])
                            d_int_dot_y_hats_2prime = np.array([])
                            v_int_dot_x_hats_2prime = np.array([])
                            v_int_dot_y_hats_2prime = np.array([])
                            d_int_dot_v_int = np.array([])

                            for k in range(len(x_hats_2prime)):
                                d_int_dot_x_hats_2prime = np.append(d_int_dot_x_hats_2prime, np.dot(d_int, x_hats_2prime[k]))
                                d_int_dot_y_hats_2prime = np.append(d_int_dot_y_hats_2prime, np.dot(d_int, y_hats_2prime[k]))

                                v_int_dot_x_hats_2prime = np.append(v_int_dot_x_hats_2prime, np.dot(v_int, x_hats_2prime[k]))
                                v_int_dot_y_hats_2prime = np.append(v_int_dot_y_hats_2prime, np.dot(v_int, y_hats_2prime[k]))

                                d_int_dot_v_int = np.append(d_int_dot_v_int, np.dot(d_int, v_int))

                            # Constants needed to compute geometry of geofence VO's
                            C1s = 1. + np.sin(phis_prime_gf) * d_int_dot_x_hats_2prime / ds_geo
                            C2s = 1. + np.cos(phis_prime_gf) * d_int_dot_y_hats_2prime / ds_geo
                            C3s = -2. * v_int_dot_x_hats_2prime - np.sin(phis_prime_gf) * d_int_dot_v_int / ds_geo
                            C4s = -2. * v_int_dot_y_hats_2prime - np.cos(phis_prime_gf) * d_int_dot_v_int / ds_geo
                            
                            # Center points of geofence VO geometries in double rotated axis system
                            Cxs_2prime = - C3s / (2. * C1s)
                            Cys_2prime = - C4s / (2. * C2s)

                            # semi major axes squared (in case of ellipse)
                            a2s = (- gs_int**2 + C2s * Cys_2prime**2) / C1s + Cxs_2prime**2
                            b2s = (- gs_int**2 + C1s * Cxs_2prime**2) / C2s + Cys_2prime**2

                            # Loop trough a2s and b2s to construct VOs, categorize them 
                            for k in range(len(a2s)):
                                # if ownship outside gf segment TODO: UPDATE FOR OUTSIDE the geofence cases!!!!!
                                if (a2s[k] <= 0):
                                    continue
                                # If Ellipse
                                elif (b2s[k] > 0):
                                    ellipse_angles = np.linspace(0., 2. * np.pi, N_angle)
                                    rotated_xs = np.sqrt(a2s[k]) * np.cos(ellipse_angles) + Cxs_2prime[k]
                                    rotated_ys = np.sqrt(b2s[k]) * np.sin(ellipse_angles) + Cys_2prime[k]
                                # If hyperbola
                                else:
                                    tmax = np.log((20. * vmax + np.sqrt(20.**2 * vmax**2 + a2s[k])) / np.sqrt(a2s[k]))
                                    tmin = -tmax
                                    if (phis_prime_gf[k] > 0):
                                        t = np.linspace(tmin, tmax, N_angle)
                                        rotated_xs = -np.sqrt(a2s[k]) * np.cosh(t) + Cxs_2prime[k]
                                    else:
                                        t = np.linspace(tmax, tmin, N_angle)
                                        rotated_xs = np.sqrt(a2s[k]) * np.cosh(t) + Cxs_2prime[k]
                                    rotated_ys = np.sqrt(-b2s[k]) * np.sinh(t) + Cys_2prime[k]
                                non_rotated_xs = rotated_xs * np.cos(phis_total_gf[k]) - rotated_ys * np.sin(phis_total_gf[k])
                                non_rotated_ys = rotated_xs * np.sin(phis_total_gf[k]) + rotated_ys * np.cos(phis_total_gf[k])

                                # Add non rotated VO's to clipper
                                xy_gf = []
                                xy_gf.append(non_rotated_xs)
                                xy_gf.append(non_rotated_ys)
                                xy_gf = np.array(xy_gf)
                                xy_gf = np.transpose(xy_gf)
                                xy_gf_tuple = tuple(map(tuple, xy_gf))

                                # Scale VO to clipper
                                VO = pyclipper.scale_to_clipper(xy_gf_tuple)
                                # Add scaled VO to clipper
                                try:
                                    pc.AddPath(VO, pyclipper.PT_CLIP, True)
                                except:
                                    pass

                    # Execute clipper command
                    FRV = pyclipper.scale_from_clipper(
                        pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO))

                    ARV = pc.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO)

                    # Make another clipper object for extra intersections
                    pc2 = pyclipper.Pyclipper()
                    # Put the ARV in there, make sure it's not empty
                    if len(ARV) > 0:
                        pc2.AddPaths(ARV, pyclipper.PT_CLIP, True)

                    # Scale back
                    ARV = pyclipper.scale_from_clipper(ARV)

                    # Check if ARV or FRV is empty
                    if len(ARV) == 0:
                        # No aircraft in the vicinity
                        # Map them into the format ARV wants. Outercircle CCW, innercircle CW
                        ARV_loc[i] = []
                        FRV_loc[i] = circle_lst
                        ARV_calc_loc[i] = []
                        # Calculate areas and store in asas
                        FRV_area_loc[i] = np.pi * (vmax ** 2 - vmin ** 2)
                        ARV_area_loc[i] = 0
                    elif len(FRV) == 0:
                        # Should not happen with one a/c or no other a/c in the vicinity.
                        # These are handled earlier. Happens when RotA has removed all
                        # Map them into the format ARV wants. Outercircle CCW, innercircle CW
                        ARV_loc[i] = circle_lst
                        FRV_loc[i] = []
                        ARV_calc_loc[i] = circle_lst
                        # Calculate areas and store in asas
                        FRV_area_loc[i] = 0
                        ARV_area_loc[i] = np.pi * (vmax ** 2 - vmin ** 2)
                    else:
                        # Check multi exteriors, if this layer is not a list, it means it has no exteriors
                        # In that case, make it a list, such that its format is consistent with further code
                        if not type(FRV[0][0]) == list:
                            FRV = [FRV]
                        if not type(ARV[0][0]) == list:
                            ARV = [ARV]
                        # Store in asas
                        FRV_loc[i] = FRV
                        ARV_loc[i] = ARV
                        # Calculate areas and store in asas
                        FRV_area_loc[i] = self.area(FRV)
                        ARV_area_loc[i] = self.area(ARV)

                        # Small ring
                        xyp = (tuple(map(tuple, np.flipud(xyc * min(vmax, ownship.tas[i] + 0.1)))),
                                tuple(map(tuple, xyc * max(vmin, ownship.tas[i] - 0.1))))
                        part = pyclipper.scale_to_clipper(xyp)
                        pc2.AddPaths(part, pyclipper.PT_SUBJECT, True)

                        # Execute clipper command
                        ARV_calc = pyclipper.scale_from_clipper(
                            pc2.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_NONZERO, pyclipper.PFT_NONZERO))
                        
                        # If no smaller ARV is found, take the full ARV
                        if len(ARV_calc) == 0:
                            ARV_calc = ARV

                        ARV_calc = ARV
                        # Update calculatable ARV for resolutions
                        ARV_calc_loc[i] = ARV_calc

        conf.FRV = FRV_loc
        conf.ARV = ARV_loc
        conf.ARV_calc = ARV_calc_loc
        conf.FRV_area = FRV_area_loc
        conf.ARV_area = ARV_area_loc
        return


    def calculate_resolution(self, conf, ownship):
        """ Calculates closest conflict-free point according to ruleset """
        # It's just linalg, however credits to: http://stackoverflow.com/a/1501725
        # Variables
        ARV = conf.ARV_calc
        gsnorth = ownship.gsnorth
        gseast = ownship.gseast
        ntraf = ownship.ntraf

        # Loop through SSDs of all aircraft
        for i in range(ntraf):
            # Only those that are in conflict need to resolve
            if conf.inconf[i] and ARV[i] is not None and len(ARV[i]) > 0:
                # Loop through all exteriors and append. Afterwards concatenate
                p = []
                q = []
                for j in range(len(ARV[i])):
                    p.append(np.array(ARV[i][j]))
                    q.append(np.diff(np.row_stack((p[j], p[j][0])), axis=0))
                p = np.concatenate(p)
                q = np.concatenate(q)
                # Calculate squared distance between edges
                l2 = np.sum(q ** 2, axis=1)
                # Catch l2 == 0 (exception)
                same = l2 < 1e-8
                l2[same] = 1.
                # Calc t
                t = np.sum((np.array([gseast[i], gsnorth[i]]) - p) * q, axis=1) / l2
                # Speed of boolean indices only slightly faster (negligible)
                # t must be limited between 0 and 1
                t = np.clip(t, 0., 1.)
                t[same] = 0.
                # Calculate closest point to each edge
                x1 = p[:, 0] + t * q[:, 0]
                y1 = p[:, 1] + t * q[:, 1]
                # Get distance squared
                d2 = (x1 - gseast[i]) ** 2 + (y1 - gsnorth[i]) ** 2
                # Sort distance
                ind = np.argsort(d2)
                x1 = x1[ind]
                y1 = y1[ind]

                conf.asase[i] = x1[0]
                conf.asasn[i] = y1[0]

            # Those that are not in conflict will be assigned zeros
            # Or those that have no solutions (full ARV)
            else:
                conf.asase[i] = 0.
                conf.asasn[i] = 0.

            # Loop through resolutions
            for i in range(ntraf):
                if (conf.asase[i] != 0. and conf.asasn[i] != 0.):
                    # calculate t_cpa for resolution
                    tres = conf.tcpamax[i]
                    dx_res = conf.asase[i] * tres
                    dy_res = conf.asasn[i] * tres
                    qdr_res = np.rad2deg(np.arctan2(dx_res, dy_res))
                    dist_res = np.sqrt(dx_res**2 + dy_res**2) / nm
                    lat_res, lon_res = geo.qdrpos(ownship.lat[i], ownship.lon[i], qdr_res, dist_res)
                    alt_res = ownship.alt[i] # [m]

                    # Check reesolution in geofence
                    geofence_defined = False
                    solution_in_geofence = True
                    try:
                        areafilter.basic_shapes['GF_' + str(ownship.id[i])]
                    except:
                        pass
                    else:
                        geofence_defined = True

                    if geofence_defined:
                        solution_in_geofence = areafilter.checkInside('GF_' + str(ownship.id[i]), lat_res, lon_res, 0)

                    # Check timeout for conflict resolution
                    current_time = time.time()
                    delta_cr_time = current_time - conflictresolutiontime.cr_time[i]
                    
                    if (delta_cr_time > 4.0 and solution_in_geofence):
                        conflictresolutiontime.cr_time[i] = current_time
                        # send resolution over mqtt
                        body = {}
                        body['ac_id'] = ownship.id[i]
                        body['timestamp'] = int(time.time())
                        body['waypoint'] = {}
                        body['waypoint']['lat'] = int(lat_res * 10**7)
                        body['waypoint']['lon'] = int(lon_res * 10**7)
                        body['waypoint']['alt'] = int(alt_res * 10**3)
                        body['tres'] = float(tres) 
                        body['vres'] = float(dist_res * nm / tres)

                        mqtt_publisher = MQTTAvoidRequestPublisher()
                        mqtt_publisher.connect(os.environ["MQTT_HOST"], int(os.environ["MQTT_PORT"]), 60)
                        mqtt_publisher.loop_start()
                        mqtt_publisher.publish('daa/avoid_request', payload=json.dumps(body))
                        mqtt_publisher.loop_stop()

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