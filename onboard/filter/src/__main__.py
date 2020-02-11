import json
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
from .conversions import meters2lat, meters2long, lat2meters, long2meters, \
                        decimal2min
# don't have nav status in here yet
# TODO handle losing sensors


class StateEstimate:
    # class for the current state estimates

    def __init__(self, lat_deg=None, lat_min=None, vel_north=None,
                 long_deg=None, long_min=None, vel_east=None, bearing_degs=None):
        self.pos = PositionDegs(lat_deg, long_deg, lat_min, long_min)
        self.vel = Velocity2D(vel_north, vel_east)
        self.bearing_degs = bearing_degs

    def asLKFInput(self):
        # Returns the state estimate as a list for filter input
        return [self.pos.lat_deg, meters2lat(self.vel.north),
                self.pos.long_deg, meters2long(self.vel.east,
                                               self.pos.lat_deg)]

    def updateFromLKF(self, numpy_array, bearing_degs):
        # Updates the list from the numpy array output of the filter
        self.pos.lat_deg = numpy_array[0]
        self.pos.long_deg = numpy_array[2]
        self.vel.north = lat2meters(numpy_array[1])
        self.vel.east = long2meters(numpy_array[3], numpy_array[0])
        self.bearing_degs = bearing_degs

    def asOdom(self):
        # Returns the current state estimate as an Odometry LCM object
        odom = Odometry()
        odom.latitude_deg, odom.latitude_min = decimal2min(self.pos.lat_deg)
        odom.longitude_deg, odom.longitude_min = decimal2min(self.pos.long_deg)
        odom.bearing_deg = self.bearing_degs
        odom.speed = self.vel.pythagorean()
        return odom


class SensorFusion:
    # Class for filtering sensor data and outputting state estimates

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
            lat_deg, lat_min = decimal2min(mean_lat)
            long_deg, long_min = decimal2min(mean_long)

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
        # temp mov_avg filter
        self.movAvg()

    def phoneCallback(self, channel, msg):
        new_phone = SensorPackage.decode(msg)
        self.phone.update(new_phone)

    def imuCallback(self, channel, msg):
        new_imu = IMU.decode(msg)
        self.imu.update(new_imu)

        # Run filter if constructed and sensors are ready
        if self.sensorsReady() and self.filter is not None:
            self.filter.run(self.gps, self.imu, self.state_estimate)
        # Construct state estimate and filter if sensors are ready
        elif self.sensorsReady():
            pos = self.gps.asDecimal()
            vel = self.gps.absolutifyVel(self.imu.bearing_degs)

            self.state_estimate = StateEstimate(pos.lat_deg, None, vel.north,
                                                pos.long_deg, None, vel.east,
                                                self.imu.bearing_degs)
            self.constructFilter()

    # def navStatusCallback(self, channel, msg):
    #     new_nav_status = NavStatus.decode(msg)
    #     self.nav_status.update(new_nav_status)

    def sensorsReady(self):
        return self.gps.ready() and self.imu.ready()

    def constructFilter(self):
        dt = self.config['dt']

        if self.config['FilterType'] == 'LinearKalman':
            x_initial = self.state_estimate
            P_initial = self.config['P_initial']
            Q = self.config['Q']
            R = self.config['R']
            self.filter = LinearKalman(x_initial, P_initial, Q, R, dt)
        else:
            print("Invalid filter type!")
            sys.exit(1)

    async def run(self):
        # Main loop for running the filter and publishing to odom
        while True:
            if self.sensorsReady() and self.filter is not None:
                odom = self.state_estimate.asOdom()
                self.lcm.publish('/odometry', odom.encode())
            await asyncio.sleep(self.config["UpdateRate"])


def main():
    fuser = SensorFusion()
    logger = Logger()
    run_coroutines(fuser.lcm.loop(), logger.lcm.loop(), fuser.run())


if __name__ == '__main__':
    main()
