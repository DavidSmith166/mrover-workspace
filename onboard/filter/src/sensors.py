import math
from abc import ABC


class RawAccelSensor(ABC):
    # Abstract class for acceleration sensors

    # Convert to absolute here or when fusing accel?
    def __init__(self):
        self.accel_x = None
        self.accel_y = None
        self.accel_z = None

    # Are acceleration sensors generic/standardized enough to allow this?
    def update(self, new_accel_sensor):
        self.accel_x = new_accel_sensor.accel_x
        self.accel_y = new_accel_sensor.accel_y
        self.accel_z = new_accel_sensor.accel_z

    # Converts acceleration to absolute components
    def absolutify(self, bearing, pitch):
        if self.accel_x is None or bearing is None or pitch is None:
            return None

        _accel_north = self.accel_x * math.cos(pitch) * math.sin(90 - bearing)
        _accel_east = self.accel_x * math.cos(pitch) * math.cos(90 - bearing)
        _accel_z = self.accel_x * math.sin(pitch)
        return Acceleration(_accel_north, _accel_east, _accel_z)

    # Separate method for pure acceleration sensors (do those even exist?)?


class RawVelSensor(ABC):
    # Abstract class for velocity sensors

    # Separate here or when fusing vel?
    def __init__(self):
        self.vel_raw = None

    def update(self, new_vel_sensor):
        self.vel_raw = new_vel_sensor.speed

    # Separates vel_raw into absolute components
    def separateAbsolute(self, bearing):
        if self.vel_raw is None or bearing is None:
            return None

        # Throw out negative velocities
        if self.vel_raw < 0:
            return None

        _vel_north = self.vel_raw * math.sin(90 - bearing)
        _vel_east = self.vel_raw * math.cos(90 - bearing)
        return Velocity(_vel_north, _vel_east, 0)


class RawPosSensor(ABC):
    # Abstract class for position sensors

    def __init__(self):
        self.lat_deg = None
        self.lat_min = None
        self.long_deg = None
        self.long_min = None

    def update(self, new_gps):
        # Updates the GPS with new LCM data
        self.lat_deg = new_gps.latitude_deg
        self.lat_min = new_gps.latitude_min
        self.long_deg = new_gps.longitude_deg
        self.long_min = new_gps.longitude_min


class RawBearingSensor(ABC):
    # Abstract class for bearing sensors
    # Moving average?

    def __init__(self):
        self.bearing = None

    def update(self, new_bearing_sensor):
        # Account for non-standardized LCM structs >:(
        if hasattr(new_bearing_sensor, 'bearing'):
            self.bearing = new_bearing_sensor.bearing
            print('bearing')
        else:
            self.bearing = new_bearing_sensor.bearing_deg
            print('bearing_deg')


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
        self.roll = None
        self.pitch = None
        self.yaw = None

    def update(self, new_imu):
        # Updates the IMU with new LCM data
        # TODO for new IMU
        pass


class RawEncoder(RawVelSensor):
    # Class for wheel encoder data
    # Does the LCM message come with data for all four encoders?
    # Should RawEncoder hold data for all four encoders?

    def __init__(self):
        RawVelSensor.__init__(self)

    def update(self, new_encoder):
        # Updates the encoder with new LCM data
        # TODO
        pass


class RawGPS(RawVelSensor, RawPosSensor, RawBearingSensor):
    # Class for GPS data

    def __init__(self):
        RawVelSensor.__init__(self)
        RawPosSensor.__init__(self)
        RawBearingSensor.__init__(self)

    def update(self, new_gps):
        # Updates the GPS with new LCM data
        RawPosSensor.update(self, new_gps)
        RawBearingSensor.update(self, new_gps)
        RawBearingSensor.update(self, new_gps)


class RawPhone(RawPosSensor, RawBearingSensor):
    # Class for burner phone data

    def __init__(self):
        RawPosSensor.__init__(self)
        RawBearingSensor.__init__(self)

    def update(self, new_phone):
        # Updates the phone with new LCM data
        RawPosSensor.update(self, new_phone)
        RawBearingSensor.update(self, new_phone)


class Acceleration:
    # Class for absolute acceleration
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z


class Velocity:
    # Class for absolute velocity
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z
