import numpy as np
import scipy.integrate
import json
from os import getenv
import random
import math

'''
Goal is to simulate the sensor inputs one could expect to see for a given path

Starts by randomly generating a number of acceleration readings under various constraints

Integrates over acceleration readings to create "true path"

Applies normal distribution to generated acceleration and gps readings

Result is a correct path and a noisy path to filter.

Stretch Goal is plot noisy, filtered, and true paths and calculate deviation of
filtered path from truth
'''


class PathGenerator:

    def __init__(self):
        # Fetch constants
        config_path = getenv('MROVER_CONFIG')
        config_path += "/config_filter/simConfig.json"
        with open(config_path, "r") as configJson:
            config = json.load(configJson)
            self.GPS_POS_STDEV_METERS = config['gps_pos_stdev_meters']
            self.GPS_VEL_STDEV_METERS = config['gps_vel_stdev_meters']
            self.GPS_BEARING_STDEV_DEGS = config['gps_bearing_stdev_degs']
            self.IMU_ACCEL_STDEV_METERS = config['imu_accel_stdev_meters']
            self.IMU_BEARING_STDEV_DEGS = config['imu_bearing_stdev_degs']
            self.IMU_PITCH_STDEV_DEGS = config['imu_bearing_stdev_degs']
            self.MAX_JERK = config['max_jerk']
            self.MAX_ACCEL = config['max_accel']
            self.MIN_ACCEL = config['min_accel']
            self.MAX_ROT_VEL_RADS = config['max_rot_vel_rads']
            self.MAX_PITCH_VEL_RADS = config['max_pitch_vel_rads']
            self.DT_S = config['dt_s']
            self.END_TIME = config['end_time']
        self.MAX_READINGS = np.int_(self.END_TIME / self.DT_S)

    # generates accelerations in m/s2
    def point_gen(self):
        last_point = random.uniform(self.MIN_ACCEL, self.MAX_ACCEL)
        num_points = 1
        yield np.float64(last_point)
        while num_points < self.MAX_READINGS:
            # new_point = random.uniform(max(last_point - self.MAX_JERK, self.MIN_ACCEL),
            #                            min(last_point + self.MAX_JERK, self.MAX_ACCEL))
            new_point = np.random.uniform(max(last_point - 0.2*self.MAX_JERK, self.MIN_ACCEL),
                                          min(last_point + self.MAX_JERK, self.MAX_ACCEL))
            num_points += 1
            last_point = new_point
            yield np.float64(new_point)

    # generates angles in radians
    def angle_gen(self, delta):
        last_angle = random.uniform(0, 2*math.pi)
        num_angles = 1
        yield np.float64(last_angle)
        while num_angles < self.MAX_READINGS:
            new_angle = random.uniform((last_angle - delta) % 2*math.pi,
                                       (last_angle + delta) % 2*math.pi)
            num_angles += 1
            last_angle = new_angle
            yield np.float64(new_angle)

    def generator(self):
        # units are m/s

        while True:

            # generate control inputs
            accel_points_x = np.fromiter(self.point_gen(), np.float64, count=self.MAX_READINGS)
            bearing_angles_rads = np.fromiter(self.angle_gen(self.MAX_ROT_VEL_RADS), np.float64,
                                              count=self.MAX_READINGS)
            pitch_angles_rads = np.fromiter(self.angle_gen(self.MAX_PITCH_VEL_RADS), np.float64,
                                            count=self.MAX_READINGS)

            # generate absolute accelerations
            bearing_sin = [math.sin(deg2rad(90) - i) for i in bearing_angles_rads]
            bearing_cos = [math.cos(deg2rad(90) - i) for i in bearing_angles_rads]
            pitch_cos = [math.cos(i) for i in pitch_angles_rads]
            # pitch_sin = [math.sin(i) for i in pitch_angles_rads]
            accel_points_north = np.multiply(accel_points_x, np.multiply(pitch_cos, bearing_sin))
            accel_points_west = np.multiply(accel_points_x, np.multiply(pitch_cos, bearing_cos))

            # generate derivatives
            vel_points_north = scipy.integrate.cumtrapz(accel_points_north, dx=self.DT_S)
            vel_points_west = scipy.integrate.cumtrapz(accel_points_west, dx=self.DT_S)
            vel_points_total = np.array([math.sqrt(vel_points_north[i]**2 + vel_points_west[i]**2)
                                         for i in range(len(vel_points_north))])

            gps_points_north = scipy.integrate.cumtrapz(vel_points_north, dx=self.DT_S)
            gps_points_north = [meters2lat(i) for i in gps_points_north]
            gps_points_west = scipy.integrate.cumtrapz(vel_points_west, dx=self.DT_S)
            gps_points_west = [meters2long(gps_points_west[i], gps_points_west[i])
                               for i in range(len(gps_points_west))]
            gps_points_north = [i + 42.277 for i in gps_points_north]
            gps_points_west = [i + 83.7382 for i in gps_points_west]

            truth = {
                "accel_x": accel_points_x,
                "accel_y": np.zeros(self.MAX_READINGS),
                "accel_z": np.zeros(self.MAX_READINGS),
                "vel_north": vel_points_north,
                "vel_west": vel_points_west,
                "vel_total": vel_points_total,
                "gps_north": gps_points_north,
                "gps_west": gps_points_west,
                "bearing": bearing_angles_rads
            }

            # noise it up
            noisy_accel_points_x = np.random.normal(accel_points_x, self.IMU_ACCEL_STDEV_METERS)
            noisy_accel_points_y = np.random.normal(np.zeros(self.MAX_READINGS), self.IMU_ACCEL_STDEV_METERS)
            noisy_accel_points_z = np.random.normal(np.zeros(self.MAX_READINGS), self.IMU_ACCEL_STDEV_METERS)

            noisy_vel_points_total = np.random.normal(vel_points_total, self.GPS_VEL_STDEV_METERS)

            # noisy_gps_points_north = np.random.normal(gps_points_north,
            #                                           meters2lat(self.GPS_POS_STDEV_METERS))
            noisy_gps_points_north = np.random.normal(gps_points_north, 0.00002)
            # noisy_gps_points_west = np.random.normal(gps_points_west, [abs(i) for i in
            #                                                            meters2long(self.GPS_POS_STDEV_METERS,
            #                                                                        gps_points_north)])
            noisy_gps_points_west = np.random.normal(gps_points_west, 0.00002)

            noisy_bearing_angles_rads = np.random.normal(bearing_angles_rads, deg2rad(self.IMU_BEARING_STDEV_DEGS))
            noisy_pitch_angles_rads = np.random.normal(pitch_angles_rads, deg2rad(self.IMU_PITCH_STDEV_DEGS))

            noisy = {
                "accel_x": noisy_accel_points_x,
                "accel_y": noisy_accel_points_y,
                "accel_z": noisy_accel_points_z,
                "vel_total": noisy_vel_points_total,
                "gps_north": noisy_gps_points_north,
                "gps_west": noisy_gps_points_west,
                "bearing": noisy_bearing_angles_rads,
                "pitch": noisy_pitch_angles_rads
            }

            delta = [abs(i) for i in gps_points_north - noisy_gps_points_north]
            print(np.max(delta))
            print(max(gps_points_north))
            delta = [abs(i) for i in vel_points_total - noisy_vel_points_total]
            print(np.max(delta))
            print(max(vel_points_total))
            delta = [abs(i) for i in accel_points_x - noisy_accel_points_x]
            print(np.max(delta))
            print(max(accel_points_x))

            yield {"truth": truth, "noisy": noisy}

    def run(self):
        return next(self.generator())


def meters2lat(meters):
    return (meters * 180) / (math.pi * 6371000)


def meters2long(meters, lat):
    if np.isscalar(lat):
        return meters2lat(meters) / math.cos((math.pi/180) * lat)
    else:
        return [meters2lat(meters) / math.cos((math.pi/180) * i) for i in lat]


def deg2rad(degs):
    return degs * math.pi / 180
