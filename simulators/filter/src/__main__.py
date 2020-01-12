from rover_common import aiolcm
from rover_msgs import IMU, GPS
import csv
from os import getenv
import time
import math
import json
import numpy.random
from .pathGenerator import PathGenerator
# TODO remove noise generation?


class Simulator:

    def __init__(self):
        self.path_generator = PathGenerator()
        path_out = self.path_generator.run()
        self.truth = path_out['truth']
        self.noisy = path_out['noisy']

        self.lcm = aiolcm.AsyncLCM()

        self.gps_millis = time.time() * 1000
        # self.phone_millis = time.time() * 1000
        self.imu_millis = time.time() * 1000
        # self.nav_status_millis = time.time() * 1000

        # Fetch constants
        config_path = getenv('MROVER_CONFIG')
        config_path += "/config_filter/simConfig.json"
        with open(config_path, "r") as configJson:
            config = json.load(configJson)
            self.gps_pos_stdev_meters = config['gps_pos_stdev_meters']
            self.gps_vel_stdev_meters = config['gps_vel_stdev_meters']
            self.gps_bearing_stdev_degs = config['gps_bearing_stdev_degs']
            self.gps_update_rate_s = config['gps_update_rate_s']
            self.imu_accel_stdev_meters = config['imu_accel_stdev_meters']
            self.imu_bearing_stdev_degs = config['imu_bearing_stdev_degs']
            self.imu_update_rate_s = config['imu_update_rate_s']
            self.CSV_MODE = config['csv_mode']

    def recordTruth(self):
        with open('truthLog.csv', mode=self.CSV_MODE) as log:
            writer = csv.writer(log)
            writer.writerow(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing', 'speed'])
            for i in len(self.truth['accel_north']):
                lat_min, lat_deg = math.modf(self.truth['gps_north'][i])
                long_min, long_deg = math.modf(self.truth['gps_west'][i])
                bearing = self.truth['bearing'][i]
                speed = self.truth['vel_total'][i]
                writer.writerow([lat_deg, lat_min, long_deg, long_min, bearing, speed])

    def sendTimestep(self):
        pass

    # # Sends noisy GPS data over /gps LCM
    # def sendGps(self):
    #     # Simulate sensor update rate
    #     if (time.time()*1000 - self.gps_millis) < self.gps_update_rate_s:
    #         return

    #     gps = GPS()

    #     # Noise up the latitude and longitude
    #     noisy_lat = numpy.random.normal(self.lat_deg,
    #                                     meters2lat(self.gps_pos_stdev_meters))
    #     gps.latitude_min, gps.latitude_deg = math.modf(noisy_lat)
    #     gps.latitude_deg = int(gps.latitude_deg)
    #     gps.latitude_min *= 60

    #     noisy_long = numpy.random.normal(self.long_deg,
    #                                      meters2long(self.gps_pos_stdev_meters,
    #                                                  self.lat_deg))
    #     gps.longitude_min, gps.longitude_deg = math.modf(noisy_long)
    #     gps.longitude_deg = int(gps.longitude_deg)
    #     gps.longitude_min *= 60

    #     gps.bearing_deg = numpy.random.normal(self.bearing,
    #                                           self.gps_bearing_stdev_degs)
    #     # abs value speed?
    #     gps.speed = numpy.random.normal(pythagorean(self.vel_north,
    #                                                 self.vel_west),
    #                                     self.gps_vel_stdev_meters)

    #     self.lcm.publish('/gps', gps.encode())
    #     self.gps_millis = time.time()*1000

    # def sendImu(self):
    #     # Simulate sensor update rate
    #     if (time.time()*1000 - self.imu_millis) < self.imu_update_rate_s:
    #         return

    #     imu = IMU()

    #     imu.accel_x = numpy.random.normal(accelAbs2x(self.bearing, self.pitch,
    #                                                  self.accel_north,
    #                                                  self.accel_west,
    #                                                  self.accel_z),
    #                                       self.imu_accel_stdev_meters)
    #     imu.accel_y = numpy.random.normal(0, self.imu_accel_stdev_meters)
    #     imu.accel_z = numpy.random.normal(0, self.imu_accel_stdev_meters)
    #     imu.gyro_x = imu.gyro_y = imu.gyro_z = imu.mag_x = imu.mag_y = \
    #         imu.mag_z = 0
    #     imu.bearing = numpy.random.normal(self.bearing, self.imu_bearing_stdev_degs)

    #     self.lcm.publish('/imu', imu.encode())
    #     self.imu_millis = time.time()*1000


def pythagorean(a, b):
    return math.sqrt(a*a + b*b)


# returns the x acceleration from absolute accelerations
def accelAbs2x(bearing, pitch, north, west, z):
    # TODO check degrees or radians for bearing/pitch
    # account for divide by 0
    # cos(pitch) != 0
    if pitch % 180 != 0:
        return z / math.sin(pitch)
    # sin(90 - bearing) != 0
    elif (90 - bearing) % 180 != 0:
        return north / (math.cos(pitch) * math.sin(90 - bearing))
    else:
        return -west / (math.cos(pitch) * math.cos(90 - bearing))


# for the dumbass linter
def main():
    sim = Simulator()


if __name__ == '__main__':
    main()
