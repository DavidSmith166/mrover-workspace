import math
from abc import ABC

class AccelSensor(ABC):
    #Abstract class for acceleration sensors

    #Convert to absolute here or when fusing accel?
    def __init__(self):
        self._accel_x = None
        self._accel_y = None
        self._accel_z = None

    #Converts _accel_x and _accel_y to absolute components
    def absolutify(self, bearing, pitch):
        #TODO
        pass
    
    #Separate method for pure acceleration sensors?


class VelSensor(ABC):
    #Abstract class for velocity sensors

    #Separate here or when fusing vel?
    def __init__(self):
        self._vel_raw = None
    
    #Separates _vel_raw into absolute components
    def separateAbsolute(self, bearing):
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
    #TODO

    #num_prev_bearings?
    def __init__(self):
        AccelSensor.__init__(self)
        BearingSensor.__init__(self)
        self._gyro_x = None
        self._gyro_y = None
        self._gyro_z = None
        self._mag_x = None
        self._mag_y = None
        self._mag_z = None


class Encoder(VelSensor):
    #Class for wheel encoder data
    #TODO

    def __init__(self):
        VelSensor.__init__(self)


class GPS(VelSensor, PosSensor, BearingSensor):
    #Class for GPS data
    #TODO

    def __init__(self):
        VelSensor.__init__(self)
        PosSensor.__init__(self)
        BearingSensor.__init__(self)


class Phone(PosSensor, BearingSensor):
    #Class for burner phone data
    #TODO

    def __init__(self):
        PosSensor.__init__(self)
        BearingSensor.__init__(self)


class RTK(PosSensor):
    #Class for RTK data
    #TODO

    def __init(self):
        PosSensor.__init__(self)