import json
# import math
# import time
# import sys
from os import getenv

from rover_common import aiolcm
import asyncio
from rover_common.aiohelper import run_coroutines
from .rawmessages import RawIMU, RawGPS, RawSensorPackage, RawNavStatus
from .filterObjects import NavState, Odom, LocationEstimate, \
                           BearingEstimate, Acceleration, Velocity
# temp
from .logger import Logger


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
                                                pos.long_deg, None, vel.west,
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
            # TODO: better error handling
            pass

    async def run(self):
        # Main loop for running the filter and publishing to odom
        while True:
            if self.sensorsReady() and self.filter is not None:
                odom = self.state_estimate.asOdom()
                self.lcm.publish('/odometry', odom.encode())
            await asyncio.sleep(self.config["UpdateRate"])


def main():
    lcm_ = aiolcm.AsyncLCM()
    filter_ = SensorFusion()

    lcm_.subscribe("/gps", filter_.gps_callback)
    lcm_.subscribe("/imu", filter_.imu_callback)
    lcm_.subscribe("/nav_status", filter_.nav_status_callback)
    lcm_.subscribe("/sensor_package", filter_.sensor_package_callback)

    # temp
    logger = Logger()

    run_coroutines(lcm_.loop(), logger.lcm.loop(), filter_.calc_odom(lcm_))


if __name__ == '__main__':
    main()
