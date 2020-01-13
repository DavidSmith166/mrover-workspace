from rover_common import aiolcm
from rover_msgs import IMU, GPS
import csv
from os import getenv
import time
import math
import json
from .pathGenerator import PathGenerator


class Simulator:

    def __init__(self):
        self.path_generator = PathGenerator()
        path_out = self.path_generator.run()
        self.truth = path_out['truth']
        self.noisy = path_out['noisy']

        self.lcm = aiolcm.AsyncLCM()

        self.millis = time.time() * 1000
        self.timesteps = 0

        # Fetch constants
        config_path = getenv('MROVER_CONFIG')
        config_path += "/config_filter/simConfig.json"
        with open(config_path, "r") as configJson:
            config = json.load(configJson)
            self.GPS_UPDATE_RATE_MILLIS = config['gps_update_rate_millis']
            self.IMU_UPDATE_RATE_MILLIS = config['imu_update_rate_millis']
            self.DT_MILLIS = config['dt_s'] * 1000
            self.CSV_MODE = config['csv_mode']
        self.GPS_DTS = int(self.GPS_UPDATE_RATE_MILLIS / self.DT_MILLIS)
        self.IMU_DTS = int(self.IMU_UPDATE_RATE_MILLIS / self.DT_MILLIS)

    def recordTruth(self):
        with open('truthLog.csv', mode=self.CSV_MODE) as log:
            writer = csv.writer(log)
            writer.writerow(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing', 'speed'])
            for i in range(len(self.truth['gps_north'])):
                lat_min, lat_deg = math.modf(self.truth['gps_north'][i])
                lat_min *= 60
                long_min, long_deg = math.modf(self.truth['gps_west'][i])
                long_min *= 60
                bearing = self.truth['bearing'][i]
                speed = self.truth['vel_total'][i]
                writer.writerow([lat_deg, lat_min, long_deg, long_min, bearing, speed])

    def sendTimestep(self):
        # send at update rate
        if (time.time() * 1000 - self.millis) < self.DT_MILLIS:
            return

        imu = IMU()
        gps = GPS()

        imu.accel_x = self.noisy['accel_x'][self.timesteps]
        imu.accel_y = self.noisy['accel_y'][self.timesteps]
        imu.accel_z = self.noisy['accel_z'][self.timesteps]
        imu.gyro_x = imu.gyro_y = imu.gyro_z = imu.mag_x = imu.mag_y = \
            imu.mag_z = 0
        imu.bearing = self.noisy['bearing'][self.timesteps]
        imu.pitch = self.noisy['pitch'][self.timesteps]

        gps.latitude_min, gps.latitude_deg = math.modf(self.noisy['gps_north'][self.timesteps])
        gps.latitude_deg = int(gps.latitude_deg)
        gps.latitude_min *= 60
        gps.longitude_min, gps.longitude_deg = math.modf(self.noisy['gps_west'][self.timesteps])
        gps.longitude_deg = int(gps.longitude_deg)
        gps.longitude_min *= 60
        gps.bearing_deg = self.noisy['bearing'][self.timesteps] * 180 / math.pi
        gps.speed = self.noisy['vel_total'][self.timesteps]

        if self.timesteps % self.GPS_DTS == 0:
            self.lcm.publish('/gps', gps.encode())
            # print('Sending GPS')

        if self.timesteps % self.IMU_DTS == 0:
            self.lcm.publish('/imu', imu.encode())
            # print('Sending IMU')

        self.millis = time.time() * 1000
        self.timesteps += 1

    def run(self):
        while self.timesteps < len(self.noisy['gps_north']):
            self.sendTimestep()

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


# for the dumbass linter
def main():
    sim = Simulator()
    sim.recordTruth()
    sim.run()


if __name__ == '__main__':
    main()
