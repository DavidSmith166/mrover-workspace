import math
from abc import ABC
from .conversions import min2decimal, deg2rad


class RawAccelSensor(ABC):
    # Abstract class for acceleration sensors

    def __init__(self):
        self.accel_x = None
        self.accel_y = None
        self.accel_z = None

    # Are acceleration sensors generic/standardized enough to allow this?
    def update(self, new_accel_sensor):
        self.accel_x = new_accel_sensor.accel_x
        self.accel_y = new_accel_sensor.accel_y
        self.accel_z = new_accel_sensor.accel_z

    def ready(self):
        return self.accel_x is not None and self.accel_y is not None and \
            self.accel_z is not None

    # Converts acceleration to absolute components
    def absolutifyAccel(self, bearing_degs, pitch_degs):
        if self.accel_x is None or bearing_degs is None or pitch_degs is None:
            return None

        accel_north = self.accel_x * math.cos(deg2rad(pitch_degs)) * \
            math.cos(deg2rad(bearing_degs))
        accel_west = -(self.accel_x * math.cos(deg2rad(pitch_degs)) *
                       math.sin(deg2rad(bearing_degs)))
        accel_z = self.accel_x * math.sin(deg2rad(pitch_degs))
        return Acceleration(accel_north, accel_west, accel_z)


class RawVelSensor(ABC):
    # Abstract class for velocity sensors

    def __init__(self):
        self.vel_raw = None

    def update(self, new_vel_sensor):
        if new_vel_sensor.speed >= 0:
            self.vel_raw = new_vel_sensor.speed

    def ready(self):
        return self.vel_raw is not None

    # Separates vel_raw into absolute components
    def absolutifyVel(self, bearing_degs):
        if self.vel_raw is None or bearing_degs is None:
            return None

        # Throw out negative velocities
        if self.vel_raw < 0:
            return None

        vel_north = self.vel_raw * math.cos(deg2rad(bearing_degs))
        vel_west = -(self.vel_raw * math.sin(deg2rad(bearing_degs)))
        return Velocity2D(vel_north, vel_west)


class RawPosSensor(ABC):
    # Abstract class for position sensors

    def __init__(self):
        self.lat_deg = None
        self.lat_min = None
        self.long_deg = None
        self.long_min = None

    def update(self, new_pos_sensor):
        self.lat_deg = new_pos_sensor.latitude_deg
        self.lat_min = new_pos_sensor.latitude_min
        self.long_deg = new_pos_sensor.longitude_deg
        self.long_min = new_pos_sensor.longitude_min

    def ready(self):
        return self.lat_deg is not None and self.lat_min is not None and \
            self.long_deg is not None and self.long_min is not None

    def asDecimal(self):
        return PositionDegs(self.lat_deg, self.long_deg, self.lat_min, self.long_min)


class RawBearingSensor(ABC):
    # Abstract class for bearing sensors

    def __init__(self):
        self.bearing_degs = None

    def ready(self):
        return self.bearing_degs is not None

    def update(self, new_bearing_sensor):
        # Account for non-standardized LCM structs >:(
        if hasattr(new_bearing_sensor, 'bearing'):
            # TODO check for degrees vs radians
            self.bearing_degs = new_bearing_sensor.bearing
        else:
            self.bearing_degs = new_bearing_sensor.bearing_deg


class RawIMU(RawAccelSensor, RawBearingSensor):
    # Class for IMU data

    def __init__(self):
        RawAccelSensor.__init__(self)
        RawBearingSensor.__init__(self)
        self.gyro_x = None
        self.gyro_y = None
        self.gyro_z = None
        self.mag_x = None
        self.mag_y = None
        self.mag_z = None
        self.roll_degs = None
        self.pitch_degs = None
        self.yaw_degs = None

    def update(self, new_imu):
        # Updates the IMU with new LCM data
        RawAccelSensor.update(self, new_imu)
        RawBearingSensor.update(self, new_imu)
        self.gyro_x = new_imu.gyro_x
        self.gyro_y = new_imu.gyro_y
        self.gyro_z = new_imu.gyro_z
        self.mag_x = new_imu.mag_x
        self.mag_y = new_imu.mag_y
        self.mag_z = new_imu.mag_z
        # TODO check for degrees or radians
        # TODO add roll and yaw
        self.pitch_degs = new_imu.pitch

    def ready(self):
        # TODO add roll and yaw
        return RawAccelSensor.ready(self) and \
            RawBearingSensor.ready(self) and self.gyro_x is not None and \
            self.gyro_y is not None and self.gyro_z is not None and \
            self.mag_x is not None and self.mag_y is not None and \
            self.mag_z is not None and self.pitch_degs is not None


class RawEncoder(RawVelSensor):
    # Class for wheel encoder data

    def __init__(self):
        RawVelSensor.__init__(self)

    def update(self, new_encoder):
        # Updates the encoder with new LCM data
        pass


class RawGPS(RawVelSensor, RawPosSensor, RawBearingSensor):
    # Class for GPS data

    def __init__(self):
        RawVelSensor.__init__(self)
        RawPosSensor.__init__(self)
        RawBearingSensor.__init__(self)

    def update(self, new_gps):
        # Updates the GPS with new LCM data
        RawVelSensor.update(self, new_gps)
        RawPosSensor.update(self, new_gps)
        RawBearingSensor.update(self, new_gps)

    def ready(self):
        return RawVelSensor.ready(self) and RawPosSensor.ready(self) \
            and RawBearingSensor.ready(self)


class RawPhone(RawPosSensor, RawBearingSensor):
    # Class for burner phone data

    def __init__(self):
        RawPosSensor.__init__(self)
        RawBearingSensor.__init__(self)

    def update(self, new_phone):
        # Updates the phone with new LCM data
        RawPosSensor.update(self, new_phone)
        RawBearingSensor.update(self, new_phone)

    def ready(self):
        return RawPosSensor.ready(self) and RawAccelSensor.ready(self)


class Acceleration:
    # Class for absolute acceleration
    def __init__(self, north, west, z):
        self.north = north
        self.west = west
        self.z = z


class Velocity2D:
    # Class for absolute velocity
    def __init__(self, north, west):
        self.north = north
        self.west = west

    def pythagorean(self):
        return math.sqrt(self.north**2 + self.west**2)


class PositionDegs:
    # Class for position in decimal degrees
    def __init__(self, lat_deg, long_deg, lat_min=0, long_min=0):
        if lat_deg is None or lat_min is None or \
           long_deg is None or long_min is None:
            self.lat_deg = lat_deg
            self.long_deg = long_deg
        else:
            self.lat_deg = min2decimal(lat_deg, lat_min)
            self.long_deg = min2decimal(long_deg, long_min)


class RawNavStatus:
    # Class for nav status

    def __init__(self):
        self.nav_status = None

    def update(self, new_nav_status):
        self.nav_status = new_nav_status.nav_state_name
