from rover_common import aiolcm
from rover_msgs import IMU, GPS
import time
import math
import numpy.random


class Simulator:
    # TODO add state generation based on acceleration

    def __init__(self):
        self.lcm = aiolcm.AsyncLCM()

        self.gps_millis = time.time() * 1000
        # self.phone_millis = time.time() * 1000
        self.imu_millis = time.time() * 1000
        # self.nav_status_millis = time.time() * 1000

        # TODO throw in a config file
        self.gps_deg_stdev_meters = 200
        self.gps_vel_stdev_meters = 50
        self.gps_bearing_stdev = 15
        self.gps_update_rate = 0.05
        self.imu_accel_stdev = 0.8
        self.imu_bearing_stdev = 4
        self.imu_update_rate = 0.01

        self.accel_north = 0
        self.accel_east = 0
        self.accel_z = 0
        self.vel_north = 0
        self.vel_east = 0
        self.lat_deg = 0
        self.long_deg = 0
        self.bearing = 0
        self.pitch = 0

    def sendGps(self):
        # Simulate sensor update rate
        if (time.time()*1000 - self.gps_millis) < self.gps_update_rate:
            return

        gps = GPS()

        noisy_lat = numpy.random.normal(self.lat_deg,
                                        meters2lat(self.gps_deg_stdev_meters))
        gps.latitude_min, gps.latitude_deg = math.modf(noisy_lat)
        gps.latitude_deg = int(gps.latitude_deg)
        gps.latitude_min *= 60

        noisy_long = numpy.random.normal(self.long_deg,
                                         meters2long(self.gps_deg_stdev_meters,
                                                     self.lat_deg))
        gps.longitude_min, gps.longitude_deg = math.modf(noisy_long)
        gps.longitude_deg = int(gps.longitude_deg)
        gps.longitude_min *= 60

        gps.bearing_deg = numpy.random.normal(self.bearing,
                                              self.gps_bearing_stdev)
        # abs value speed?
        gps.speed = numpy.random.normal(pythagorean(self.vel_north,
                                                    self.vel_east),
                                        self.gps_vel_stdev_meters)

        self.lcm.publish('/gps', gps.encode())
        self.gps_millis = time.time()*1000

    def sendImu(self):
        # Simulate sensor update rate
        if (time.time()*1000 - self.imu_millis) < self.imu_update_rate:
            return

        imu = IMU()

        imu.accel_x = numpy.random.normal(accelAbs2x(self.bearing, self.pitch,
                                                     self.accel_north,
                                                     self.accel_east,
                                                     self.accel_z),
                                          self.imu_accel_stdev)
        imu.accel_y = numpy.random.normal(0, self.imu_accel_stdev)
        imu.accel_z = numpy.random.normal(0, self.imu_accel_stdev)
        imu.gyro_x = imu.gyro_y = imu.gyro_z = imu.mag_x = imu.mag_y = \
            imu.mag_z = 0
        imu.bearing = numpy.random.normal(self.bearing, self.imu_bearing_stdev)

        self.lcm.publish('/imu', imu.encode())
        self.imu_millis = time.time()*1000

    # TODO trash program design, just wanted to get something running
    def simStatic(self):
        self.lat_deg = 42.27700000
        self.long_deg = 83.73820000

        while True:
            self.sendGps()
            self.sendImu()


def meters2lat(meters):
    return (meters * 180) / (math.pi * 6371000)


def meters2long(meters, lat):
    return meters2lat(meters) / math.cos((math.pi/180) * lat)


def pythagorean(a, b):
    return math.sqrt(a*a + b*b)


# returns the x acceleration from absolute accelerations
def accelAbs2x(bearing, pitch, north, east, z):
    # TODO check degrees or radians for bearing/pitch
    # account for divide by 0
    # cos(pitch) != 0
    if pitch % 180 != 0:
        return z / math.sin(pitch)
    # sin(90 - bearing) != 0
    elif (90 - bearing) % 180 != 0:
        return north / (math.cos(pitch) * math.sin(90 - bearing))
    else:
        return east / (math.cos(pitch) * math.cos(90 - bearing))


# for the dumbass linter
def main():
    sim = Simulator()
    sim.simStatic()


if __name__ == '__main__':
    main()
