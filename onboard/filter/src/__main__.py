import json
import math
# import time
import sys
from os import getenv

from rover_common import aiolcm
import asyncio
from rover_common.aiohelper import run_coroutines
from rover_msgs import IMU, GPS, NavStatus, SensorPackage, \
                        Odometry
from .inputs import RawGPS, RawPhone, RawIMU, Acceleration, \
                     Velocity2D, PositionDegs, RawNavStatus
from .logger import Logger
from .linearKalman import LinearKalman


class StateEstimate:
    # Abstract class for the current state estimate

    def __init__(self, lat_deg=None, vel_north=None, long_deg=None,
                 vel_east=None, bearing=None):
        self.pos = PositionDegs(lat_deg, long_deg)
        self.vel = Velocity2D(vel_north, vel_east)
        self.bearing = bearing

    def asFilterInput(self):
        # Returns the state estimate as a list for filter input
        return [self.pos.lat_degs, self.vel.north,
                self.pos.long_deg, self.vel.east]

    def updateFromNumpy(self, numpy_array, bearing):
        # Updates the list from the numpy array output of the filter
        self.pos.lat_deg = numpy_array[0]
        self.pos.long_deg = numpy_array[2]
        self.vel.north = numpy_array[1]
        self.vel.east = numpy_array[3]

    def asOdom(self):
        # Returns the current state estimate as an Odometry LCM object
        odom = Odometry()
        odom.latitude_min, odom.latitude_deg = math.modf(self.lat_deg)
        odom.latitude_deg = int(self.latitude_deg)
        odom.latitude_min *= 60
        odom.longitude_min, odom.longitude_deg = math.modf(self.long_deg)
        odom.longitude_deg = int(self.longitude_deg)
        odom.longitude_min *= 60
        odom.bearing_deg = self.bearing
        odom.speed = self.vel.pythagorean()
        return odom


class SensorFusion:
    # Class for filtering sensor data and outputting state estimates
    # TODO: rename to MemeTeam

    def __init__(self):
        # Read in options from logConfig
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
        self.lcm.subscribe("/gps", self.gps.callback)
        self.lcm.subscribe("/imu", self.imu.callback)
        # self.lcm.subscribe("/nav_status", self.nav_status.callback)
        self.lcm.subscribe("/sensor_package", self.phone.callback)
    
    def gpsCallback(self, channel, msg):
        new_gps = GPS.decode(msg)
        self.gps.update(new_gps)
        self.gps.valid = True

    def phoneCallback(self, channel, msg):
        new_phone = SensorPackage.decode(msg)
        self.phone.update(new_phone)
        self.phone.valid = True
    
    def imuCallback(self, channel, msg):
        new_imu = IMU.decode(msg)
        self.imu.update(new_imu)
        self.imu.valid = True
        # Run filter
        if self.sensorsValid and self.filter != None:
            accel = self.imu.absolutifyAccel(self.imu.bearing,
                                                self.imu.pitch)
            u = [accel.north, accel.east]
            pos = self.gps.asDecimal()
            vel = self.gps.absolutifyVel(self.imu.bearing)
            z = [pos.lat_deg, vel.north, pos.long_deg, vel.east]
            x = self.filter.run(u, z)
            self.state_estimate.updateFromNumpy(x, self.imu.bearing)
        elif self.sensorsValid:
            # Initial estimate using GPS velocity or IMU accel?
            pos = self.gps.asDecimal()
            vel = self.gps.absolutifyVel(self.imu.bearing)
            self.state_estimate = StateEstimate(pos.lat_deg, vel.north,
                                                pos.long_deg, vel.east)
            self.constructFilter()
        else:
            pass

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
            if self.sensorsValid and self.filter != None:
                odom = self.state_estimate.asOdom()
                self.lcm.publish('/odometry', odom.encode())
            await asyncio.sleep(self.filterConfig["constants"]["updateRate"])


if __name__ == '__main__':
    fuser = SensorFusion()
    logger = Logger()
    run_coroutines(fuser.lcm.loop(), logger.lcm.loop(), fuser.run())
