import math
from abc import ABC, abstractmethod
from .conversions import min2decimal, deg2rad


class Sensor(ABC):
    # Abstract class for sensors
    def __init__(self):
        self.fresh = False

    @abstractmethod
    def update(self, new_sensor):
        self.fresh = True

    @abstractmethod
    def ready(self):
        pass


class SensorComponent(ABC):
    # Abstract class for sensor components
    @abstractmethod
    def update(self, new_sensor_component):
        pass

    @abstractmethod
    def ready(self):
        pass


class AccelComponent(SensorComponent):
    # Class for acceleration sensor component

    def __init__(self):
        self.accel_x = None
        self.accel_y = None
        self.accel_z = None

    def update(self, new_accel_sensor):
        self.accel_x = new_accel_sensor.accel_x
        self.accel_y = new_accel_sensor.accel_y
        self.accel_z = new_accel_sensor.accel_z

    def ready(self):
        return self.accel_x is not None and self.accel_y is not None and \
            self.accel_z is not None

    # Converts acceleration to absolute components
    def absolutify(self, bearing_degs, pitch_degs):
        if self.accel_x is None or bearing_degs is None or pitch_degs is None:
            return None

        accel_north = self.accel_x * math.cos(deg2rad(pitch_degs)) * \
            math.cos(deg2rad(bearing_degs))
        accel_east = self.accel_x * math.cos(deg2rad(pitch_degs)) * \
            math.sin(deg2rad(bearing_degs))
        accel_z = self.accel_x * math.sin(deg2rad(pitch_degs))
        return Acceleration(accel_north, accel_east, accel_z)


class VelComponent(SensorComponent):
    # Class for velocity sensor component

    def __init__(self):
        self.vel_raw = None

    def update(self, new_vel_sensor):
        if new_vel_sensor.speed >= 0:
            self.vel_raw = new_vel_sensor.speed

    def ready(self):
        return self.vel_raw is not None

    # Separates vel_raw into absolute components
    def absolutify(self, bearing_degs):
        if self.vel_raw is None or bearing_degs is None:
            return None

        # Throw out negative velocities
        if self.vel_raw < 0:
            return None

        vel_north = self.vel_raw * math.cos(deg2rad(bearing_degs))
        vel_east = self.vel_raw * math.sin(deg2rad(bearing_degs))
        return Velocity2D(vel_north, vel_east)


class PosComponent(SensorComponent):
    # Class for position sensor component

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


class BearingComponent(SensorComponent):
    # Class for acceleration bearing component

    def __init__(self):
        self.bearing_degs = None

    def ready(self):
        return self.bearing_degs is not None

    def update(self, new_bearing_sensor):
        # Account for non-standardized LCM structs >:(
        if hasattr(new_bearing_sensor, 'bearing'):
            self.bearing_degs = new_bearing_sensor.bearing
        else:
            self.bearing_degs = new_bearing_sensor.bearing_deg


class Imu(Sensor):
    # Class for IMU data

    def __init__(self):
        super().__init__()
        self.accel = AccelComponent()
        self.bearing = BearingComponent()
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
        self.accel.update(new_imu)
        self.bearing.update(new_imu)
        self.gyro_x = new_imu.gyro_x
        self.gyro_y = new_imu.gyro_y
        self.gyro_z = new_imu.gyro_z
        self.mag_x = new_imu.mag_x
        self.mag_y = new_imu.mag_y
        self.mag_z = new_imu.mag_z
        # TODO add roll and yaw, PAY ATTENTION TO UNITS
        self.pitch_degs = new_imu.pitch
        super().update(new_imu)

    def ready(self):
        # TODO add roll and yaw
        return self.accel.ready() and \
            self.bearing.ready() and self.gyro_x is not None and \
            self.gyro_y is not None and self.gyro_z is not None and \
            self.mag_x is not None and self.mag_y is not None and \
            self.mag_z is not None and self.pitch_degs is not None


class Gps(Sensor):
    # Class for GPS data

    def __init__(self):
        super().__init__()
        self.vel = VelComponent()
        self.pos = PosComponent()
        self.bearing = BearingComponent()

    def update(self, new_gps):
        # Updates the GPS with new LCM data
        self.vel.update(new_gps)
        self.pos.update(new_gps)
        self.bearing.update(new_gps)
        super().update(new_gps)

    def ready(self):
        return self.vel.ready() and self.pos.ready() and self.bearing.ready()


class Phone(Sensor):
    # Class for burner phone data

    def __init__(self):
        super().__init__()
        self.pos = PosComponent()
        self.bearing = BearingComponent()

    def update(self, new_phone):
        # Updates the phone with new LCM data
        self.pos.update(new_phone)
        self.bearing.update(new_phone)

    def ready(self):
        return self.pos.ready() and self.bearing.ready()


class NavStatus:
    # Class for nav status

    def __init__(self):
        self.nav_status = None

    def update(self, new_nav_status):
        self.nav_status = new_nav_status.nav_state_name


class Acceleration:
    # Class for absolute acceleration
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z


class Velocity2D:
    # Class for absolute velocity
    def __init__(self, north, east):
        self.north = north
        self.east = east

    def pythagorean(self):
        return math.sqrt(self.north**2 + self.east**2)


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
