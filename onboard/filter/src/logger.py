import csv

from rover_common import aiolcm
import asyncio
from rover_msgs import IMU, GPS, NavStatus, Odometry, \
                       SensorPackage

def run():
    #Create files and write headers
    write(['lat_deg','lat_min','long_deg','long_min','bearing','speed'], 'gps')
    write(['accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z',\
                            'mag_x','mag_y','mag_z','bearing'], 'imu')
    write(['nav_state','nav_state_name','completed_wps','missed_wps',\
                            'total_wps','found_tbs','total_tbs'], 'navStatus')
    write(['lat_deg','lat_min','long_deg','long_min','bearing','speed'], 'phone')
    write(['lat_deg','lat_min','long_deg','long_min','bearing','speed'], 'odom')

    #Subscribe to LCM channels
    #Modify to accept new sensors
    lcm = aiolcm.AsyncLCM()
    lcm.subscribe("/gps", Logger.gps_callback)
    lcm.subscribe("/imu", Logger.imu_callback)
    lcm.subscribe("/nav_status", Logger.nav_status_callback)
    lcm.subscribe("/sensor_package", Logger.sensor_package_callback)
    lcm.subscribe("/odometry", Logger.odom_callback)

def write(contents, type):
    #Writes contents to the log specified by type
    with open(type + 'Log.csv', mode='w') as log:
        writer = csv.writer(log)
        writer.writerow(contents)

def gps_callback(channel, msg):
    gps = GPS.decode(msg)
    write([gps.lat_deg,gps.lat_min,gps.long_deg,gps.long_min,\
                gps.bearing,gps.speed], 'gps')

def phone_callback(channel, msg):
    phone = SensorPackage.decode(msg)
    write([phone.lat_deg,phone.lat_min,phone.long_deg,phone.long_min,\
                phone.bearing,phone.speed], 'phone')

def imu_callback(channel, msg):
    imu = IMU.decode(msg)
    write([imu.accel_x,imu.accel_y,imu.accel_z,imu.gyro_x,imu.gyro_y,\
                imu.gyro_z,imu.mag_x,imu.mag_y,imu.mag_z,imu.bearing], 'imu')

def nav_status_callback(channel, msg):
    nav_status = NavStatus.decode(msg)
    write([nav_status.nav_state,nav_statusnav_state_name,nav_status.completed_wps,\
                nav_status.missed_wps,nav_status.total_wps,nav_status.found_tbs,\
                nav_status.total_tbs], 'navStatus')

def odom_callback(channel, msg):
    odom = Odometry.decode(msg)
    write([odom.lat_deg,odom.lat_min,odom.long_deg,odom.long_min,\
                odom.bearing,odom.speed], 'odom')

if __name__ == "__main__":
    run()