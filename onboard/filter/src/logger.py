import csv

from rover_common import aiolcm
import asyncio
from rover_msgs import IMU, GPS, NavStatus, Odometry, \
                       SensorPackage
from rover_common.aiohelper import run_coroutines
from .__main__ import SensorFusion

class Logger:
    def __init__(self):
        self.gps = None
        self.phone = None
        self.imu = None
        self.nav_status = None
        self.odom = None

        self.writer = None

        #Create files and write headers
        self.write(['lat_deg','lat_min','long_deg','long_min','bearing','speed'], 'gps')
        self.write(['accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z',\
                                'mag_x','mag_y','mag_z','bearing'], 'imu')
        self.write(['nav_state','nav_state_name','completed_wps','missed_wps',\
                                'total_wps','found_tbs','total_tbs'], 'navStatus')
        self.write(['lat_deg','lat_min','long_deg','long_min','bearing','speed'], 'phone')
        self.write(['lat_deg','lat_min','long_deg','long_min','bearing','speed'], 'odom')

        #Subscribe to LCM channels
        #Modify to accept new sensors
        lcm = aiolcm.AsyncLCM()
        lcm.subscribe("/gps", Logger.gps_callback)
        lcm.subscribe("/imu", Logger.imu_callback)
        lcm.subscribe("/nav_status", Logger.nav_status_callback)
        lcm.subscribe("/sensor_package", Logger.sensor_package_callback)
        lcm.subscribe("/odometry", Logger.odom_callback)
    
    def write(self, contents, type):
        #Writes contents to the log specified by type
        with open(type + 'Log.csv', mode='w') as log:
            self.writer = csv.writer(log)
            self.writer.writerow(contents)

    def gps_callback(self, channel, msg):
        self.gps = GPS.decode(msg)
        self.write([self.gps.lat_deg,self.gps.lat_min,self.gps.long_deg,self.gps.long_min,\
                    self.gps.bearing,self.gps.speed], 'gps')

    def phone_callback(self, channel, msg):
        self.phone = SensorPackage.decode(msg)
        self.write([self.phone.lat_deg,self.phone.lat_min,self.phone.long_deg,self.phone.long_min,\
                    self.phone.bearing,self.phone.speed], 'phone')

    def imu_callback(self, channel, msg):
        self.imu = IMU.decode(msg)
        self.write([self.imu.accel_x,self.imu.accel_y,self.imu.accel_z,self.imu.gyro_x,self.imu.gyro_y,\
                    self.imu.gyro_z,self.imu.mag_x,self.imu.mag_y,self.imu.mag_z,self.imu.bearing], 'imu')

    def nav_status_callback(self, channel, msg):
        self.nav_status = NavStatus.decode(msg)
        self.write([self.nav_status.nav_state,self.nav_statusnav_state_name,self.nav_status.completed_wps,\
                    self.nav_status.missed_wps,self.nav_status.total_wps,self.nav_status.found_tbs,\
                    self.nav_status.total_tbs], 'navStatus')

    def odom_callback(self, channel, msg):
        self.odom = Odometry.decode(msg)
        self.write([self.odom.lat_deg,self.odom.lat_min,self.odom.long_deg,self.odom.long_min,\
                    self.odom.bearing,self.odom.speed], 'odom')
