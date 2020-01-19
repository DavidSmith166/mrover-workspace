import json
import math
# import time
import sys
from os import getenv

from rover_common import aiolcm
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_msgs import IMU, GPS, SensorPackage, Odometry
from .inputs import RawGPS, RawPhone, RawIMU, Velocity2D, \
                    PositionDegs
from .logger import Logger
from .linearKalman import LinearKalman
from conversions import meters2lat, meters2long, lat2meters, long2meters
# don't have nav status in here yet
# TODO critical: confirm meters->degree math works out at all stages


class StateEstimate:
    # Abstract class for the current state estimate
    # TODO Need to go through and do better error handling

    def __init__(self, lat_deg=None, lat_min=None, vel_north=None,
                 long_deg=None, long_min=None, vel_west=None, bearing=None):
        self.pos = PositionDegs(lat_deg, long_deg)
        self.vel = Velocity2D(vel_north, vel_west)
        self.bearing = bearing

    def asFilterInput(self):
        # Returns the state estimate as a list for filter input
        return [self.pos.lat_deg, meters2lat(self.vel.north),
                self.pos.long_deg, meters2long(self.vel.west,
                                               self.pos.lat_deg)]

    def updateFromNumpy(self, numpy_array, bearing):
        # Updates the list from the numpy array output of the filter
        self.pos.lat_deg = numpy_array[0]
        self.pos.long_deg = numpy_array[2]
        self.vel.north = lat2meters(numpy_array[1])
        self.vel.west = long2meters(numpy_array[3], numpy_array[1])
        self.bearing = bearing

    def asOdom(self):
        # Returns the current state estimate as an Odometry LCM object
        odom = Odometry()
        odom.latitude_min, odom.latitude_deg = math.modf(self.pos.lat_deg)
        odom.latitude_deg = int(odom.latitude_deg)
        odom.latitude_min *= 60
        odom.longitude_min, odom.longitude_deg = math.modf(self.pos.long_deg)
        odom.longitude_deg = int(odom.longitude_deg)
        odom.longitude_min *= 60
        odom.bearing_deg = self.bearing * math.pi / 180
        odom.speed = self.vel.pythagorean()
        return odom


class SensorFusion:
    # Class for filtering sensor data and outputting state estimates
    # TODO: rename to MemeTeam

    def __init__(self):
        # Read in options from config
        config_path = getenv('MROVER_CONFIG')
        config_path += "/config_filter/config.json"
        with open(config_path, "r") as config:
            self.config = json.load(config)

        # Inputs
        self.gps = RawGPS()
        self.imu = RawIMU()
        self.phone = RawPhone()
        # self.nav_status = RawNavStatus()

        self.filter = None
        self.state_estimate = StateEstimate()

        # Build LCM
        self.lcm = aiolcm.AsyncLCM()
        self.lcm.subscribe("/gps", self.gpsCallback)
        self.lcm.subscribe("/imu", self.imuCallback)
        # self.lcm.subscribe("/nav_status", self.nav_statusCallback)
        self.lcm.subscribe("/sensor_package", self.phoneCallback)

        # Temp mov_avg filter
        self.gps_readings = []

    def movAvg(self):
        if len(self.gps_readings) >= 5:
            mean_lat = 0
            mean_long = 0
            for reading in self.gps_readings:
                mean_lat += reading.lat_deg
                mean_long += reading.long_deg
            mean_lat /= 5
            mean_long /= 5
            lat_min, lat_deg = math.modf(mean_lat)
            lat_deg = int(lat_deg)
            lat_min *= 60
            long_min, long_deg = math.modf(mean_long)
            long_deg = int(long_deg)
            long_min *= 60

            odom = Odometry()
            odom.latitude_deg = lat_deg
            odom.latitude_min = lat_min
            odom.longitude_deg = long_deg
            odom.longitude_min = long_min
            odom.bearing_deg = 0
            odom.speed = 0
            self.lcm.publish('/mov_avg', odom.encode())

            self.gps_readings.pop(0)

        self.gps_readings.append(self.gps.asDecimal())

    def gpsCallback(self, channel, msg):
        new_gps = GPS.decode(msg)
        self.gps.update(new_gps)
        self.movAvg()

    def phoneCallback(self, channel, msg):
        new_phone = SensorPackage.decode(msg)
        self.phone.update(new_phone)

    def imuCallback(self, channel, msg):
        new_imu = IMU.decode(msg)
        self.imu.update(new_imu)
        # Run filter
        if self.sensorsValid() and self.filter is not None:
            pos = self.gps.asDecimal()
            accel = self.imu.absolutifyAccel(self.imu.bearing,
                                             self.imu.pitch)
            if accel is None:
                return

            u = [meters2lat(accel.north), meters2long(accel.west, pos.lat_deg)]
            vel = self.gps.absolutifyVel(self.imu.bearing)
            z = [pos.lat_deg, meters2lat(vel.north), pos.long_deg,
                 meters2long(vel.west, pos.lat_deg)]
            x = self.filter.run(u, z)
            self.state_estimate.updateFromNumpy(x, self.imu.bearing)
        elif self.sensorsValid():
            # Initial estimate using GPS velocity or IMU accel?
            pos = self.gps.asDecimal()
            vel = self.gps.absolutifyVel(self.imu.bearing)

            self.state_estimate = StateEstimate(pos.lat_deg, vel.north,
                                                pos.long_deg, vel.west,
                                                self.imu.bearing)
            self.constructFilter()

    # def navStatusCallback(self, channel, msg):
    #     new_nav_status = NavStatus.decode(msg)
    #     self.nav_status.update(new_nav_status)

    def sensorsValid(self):
        # Do we need phone valid? IDK what we're even using the phone for lmao
        return self.gps.valid and self.imu.valid

    def constructFilter(self):
        x_initial = self.state_estimate.asFilterInput()
        P_initial = self.config['P_initial']
        Q = self.config['Q']
        R = self.config['R']
        dt = self.config['dt']

        if self.config['FilterType'] == 'LinearKalman':
            self.filter = LinearKalman(x_initial, P_initial, Q, R, dt)
        else:
            # TODO: better error handling
            sys.exit()

    async def run(self):
        # Main loop for running the filter and publishing to odom
        while True:
            if self.sensorsValid and self.filter is not None:
                odom = self.state_estimate.asOdom()
                self.lcm.publish('/odometry', odom.encode())
            await asyncio.sleep(self.config["UpdateRate"])


def main():
    fuser = SensorFusion()
    logger = Logger()
    run_coroutines(fuser.lcm.loop(), logger.lcm.loop(), fuser.run())


if __name__ == '__main__':
    main()
