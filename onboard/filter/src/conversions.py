import math
import numpy as np

# conversion functions

def meters2lat(meters):
    if np.isscalar(meters):
        return rad2deg(meters / 6371000)
    else:
        return [rad2deg(i / 6371000) for i in meters]


def meters2long(meters, lat):
    if np.isscalar(meters):
        if np.isscalar(lat):
            return meters2lat(meters) / math.cos(deg2rad(lat))
        else:
            return [meters2lat(meters) / math.cos(deg2rad(i)) for i in lat]
    else:
        if np.isscalar(lat):
            return [meters2lat(i) / math.cos(deg2rad(lat)) for i in meters]
        else:
            return [meters2lat(i) / math.cos(deg2rad(j)) for i, j in zip(meters, lat)]


def lat2meters(lat):
    if np.isscalar(lat):
        return deg2rad(lat * 6371000)
    else:
        return [deg2rad(i * 6371000) for i in lat]


def long2meters(long, lat):
    if np.isscalar(long):
        if np.isscalar(lat):
<<<<<<< HEAD
            return lat2meters(long * math.cos(deg2rad(lat)))
        else:
            return [lat2meters(long * math.cos(deg2rad(i))) for i in lat]
        if np.isscalar(lat):
            return [lat2meters(i) * math.cos(deg2rad(lat)) for i in long]
        else:
            return [lat2meters(i) * math.cos(deg2rad(j)) for i, j in zip(long, lat)]
>>>>>>> Clean up LinearKalman. Move meters to degree conversion out of config. Move conversion functions to conversions.py. Change valid bool in inputs to ready() function.


def deg2rad(deg):
        return deg * math.pi / 180
    else:
        return [i * math.pi / 180 for i in deg]


def rad2deg(rad):
    if np.isscalar(rad):
<<<<<<< HEAD
        return math.degrees(rad)
    else:
        return [math.degrees(i) for i in rad]


