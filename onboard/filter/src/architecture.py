import math
from abc import ABC

class AccelSensor(ABC):
    #Abstract class for acceleration sensors

    def __init__(self):
        self._accel_x = None
        self._accel_y = None
        self._accel_z = None
    
    #Separate method for pure acceleration sensors?


class VelSensor(ABC):
    #Abstract class for velocity sensors

    def __init__(self):
        self._vel_x = None
        self._vel_y = None
    
    #Separates speed into its x-y components
    def separate(self, raw_speed, bearing):
        #TODO
        pass


class PosSensor(ABC):
    #Abstract class for position sensors

    def __init__(self):
        self._lat_deg = None
        self._lat_min = None
        self._long_deg = None
        self._long_min = None


class BearingSensor(ABC):
    #Abstract class for bearing sensors

    def __init__(self):
        self._bearing = None


class IMU(AccelSensor, BearingSensor):
    #Class for IMU data

    def __init__(self):
        AccelSensor.__init__(self)
        BearingSensor.__init__(self)
        self._gyro_x = None
        self._gyro_y = None
        self._gyro_z = None
        self._mag_x = None
        self._mag_y = None
        self._mag_z = None

