import csv
import json
import os
import time

from rover_common import aiolcm
from rover_msgs import IMU, GPS, NavStatus, Odometry, \
                       SensorPackage


class Logger:

    def __init__(self):
        # Read in options from logConfig
        config_path = os.getenv('MROVER_CONFIG')
        config_path += "/config_filter/logConfig.json"
        with open(config_path, "r") as config:
            self.logConfig = json.load(config)

        config_path = os.getenv('MROVER_CONFIG')
        config_path += "/config_filter/config.json"
        with open(config_path, "r") as config:
            self.filterConfig = json.load(config)

        # Create files and write headers
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'gps')
        self.write(['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y',
                    'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'bearing'], 'imu')
        self.write(['nav_state', 'nav_state_name', 'completed_wps',
                    'missed_wps', 'total_wps', 'found_tbs', 'total_tbs'],
                   'navStatus')
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'phone')
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'odom')
        self.write(['lat_deg', 'lat_min', 'long_deg', 'long_min', 'bearing',
                    'speed'], 'movAvg')
        self.write(['Q', 'FilterType', 'P_initial', 'R', 'dt', 'UpdateRate'], 'config')

        P_initial_str = str(self.filterConfig['P_initial']).replace(',', ' ')
        R_str = str(self.filterConfig['R']).replace(',', ' ')
        self.write([self.filterConfig['Q'], self.filterConfig['FilterType'],
                    P_initial_str, R_str, self.filterConfig['dt'],
                    self.filterConfig['UpdateRate']], 'config')

        # Subscribe to LCM channels
        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self.gps_callback)
        self.lcm.subscribe("/imu", self.imu_callback)
        self.lcm.subscribe("/nav_status", self.nav_status_callback)
        self.lcm.subscribe("/sensor_package", self.phone_callback)
        self.lcm.subscribe("/odometry", self.odom_callback)
        # Temp mov_avg filter
        self.lcm.subscribe("/mov_avg", self.mov_avg_callback)

        # Initialize sensor timestamps
        self.gps_millis = time.time() * 1000
        self.phone_millis = time.time() * 1000
        self.imu_millis = time.time() * 1000
        self.nav_status_millis = time.time() * 1000
        self.odom_millis = time.time() * 1000
        # Temp mov_avg filter
        self.mov_avg_millis = time.time() * 1000

    def write(self, contents, type):
        # Writes contents to the log specified by type
        # with open(self.file_path + type + 'Log.csv', 'w') as log:
        with open(os.path.join('onboard', 'filter', 'logs', type + 'Log.csv'),
                  mode=self.logConfig['mode']) as log:
            writer = csv.writer(log)
            writer.writerow(contents)

    def gps_callback(self, channel, msg):
        gps = GPS.decode(msg)
        if (time.time()*1000 - self.gps_millis) > \
                self.logConfig['rate_millis']['gps']:
            self.write([gps.latitude_deg, gps.latitude_min, gps.longitude_deg,
                        gps.longitude_min, gps.bearing_deg, gps.speed], 'gps')
            self.gps_millis = time.time()*1000

    def phone_callback(self, channel, msg):
        phone = SensorPackage.decode(msg)
        if (time.time()*1000 - self.phone_millis) > \
                self.logConfig['rate_millis']['phone']:
            self.write([phone.latitude_deg, phone.latitude_min,
                        phone.longitude_deg, phone.longitude_min,
                        phone.bearing, phone.speed], 'phone')
            self.phone_millis = time.time()*1000

    def imu_callback(self, channel, msg):
        imu = IMU.decode(msg)
        if (time.time()*1000 - self.imu_millis) > \
                self.logConfig['rate_millis']['imu']:
            self.write([imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x,
                        imu.gyro_y, imu.gyro_z, imu.mag_x, imu.mag_y,
                        imu.mag_z, imu.bearing], 'imu')
            self.imu_millis = time.time()*1000

    def nav_status_callback(self, channel, msg):
        nav_status = NavStatus.decode(msg)
        if (time.time()*1000 - self.nav_status_millis) > \
                self.logConfig['rate_millis']['nav_status']:
            self.write([nav_status.nav_state, nav_status.nav_state_name,
                        nav_status.completed_wps, nav_status.missed_wps,
                        nav_status.total_wps, nav_status.found_tbs,
                        nav_status.total_tbs], 'navStatus')
            self.nav_status_millis = time.time()*1000

    def odom_callback(self, channel, msg):
        odom = Odometry.decode(msg)
        if (time.time()*1000 - self.odom_millis) > \
                self.logConfig['rate_millis']['odom']:
            self.write([odom.latitude_deg, odom.latitude_min,
                        odom.longitude_deg, odom.longitude_min,
                        odom.bearing_deg, odom.speed], 'odom')
            self.odom_millis = time.time()*1000

    def mov_avg_callback(self, channel, msg):
        mov_avg = Odometry.decode(msg)
        if (time.time()*1000 - self.mov_avg_millis) > \
                self.logConfig['rate_millis']['odom']:
            self.write([mov_avg.latitude_deg, mov_avg.latitude_min,
                        mov_avg.longitude_deg, mov_avg.longitude_min,
                        mov_avg.bearing_deg, mov_avg.speed], 'movAvg')
            self.mov_avg_millis = time.time()*1000


if __name__ == "__main__":
    logger = Logger()
