import csv

from rover_common import aiolcm
# from rover_common.aiohelper import run_coroutines
from rover_msgs import IMU, GPS, NavStatus, Odometry, \
                       SensorPackage


class Logger:
    # TODO: write vs. append mode, record frequency

    def __init__(self):
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

        # Subscribe to LCM channels
        # Modify to accept new sensors
        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self.gps_callback)
        self.lcm.subscribe("/imu", self.imu_callback)
        self.lcm.subscribe("/nav_status", self.nav_status_callback)
        self.lcm.subscribe("/sensor_package", self.phone_callback)
        self.lcm.subscribe("/odometry", self.odom_callback)

    def write(self, contents, type):
        # Writes contents to the log specified by type
        with open(type + 'Log.csv', mode='a') as log:
            writer = csv.writer(log)
            writer.writerow(contents)

    def gps_callback(self, channel, msg):
        gps = GPS.decode(msg)
        self.write([gps.lat_deg, gps.lat_min, gps.long_deg, gps.long_min,
                    gps.bearing, gps.speed], 'gps')

    def phone_callback(self, channel, msg):
        phone = SensorPackage.decode(msg)
        self.write([phone.latitude_deg, phone.latitude_min,
                    phone.longitude_deg, phone.longitude_min, phone.bearing,
                    phone.speed], 'phone')

    def imu_callback(self, channel, msg):
        imu = IMU.decode(msg)
        self.write([imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x,
                    imu.gyro_y, imu.gyro_z, imu.mag_x, imu.mag_y, imu.mag_z,
                    imu.bearing], 'imu')

    def nav_status_callback(self, channel, msg):
        nav_status = NavStatus.decode(msg)
        self.write([nav_status.nav_state, nav_status.nav_state_name,
                    nav_status.completed_wps, nav_status.missed_wps,
                    nav_status.total_wps, nav_status.found_tbs,
                    nav_status.total_tbs], 'navStatus')

    def odom_callback(self, channel, msg):
        odom = Odometry.decode(msg)
        self.write([odom.latitude_deg, odom.latitude_min,
                    odom.longitude_deg, odom.longitude_min,
                    odom.bearing_deg, odom.speed], 'odom')


if __name__ == "__main__":
    logger = Logger()
