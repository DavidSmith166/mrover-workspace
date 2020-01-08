import numpy as np
import random

'''
Goal is to simulate the sensor inputs one could expect to see for a given path

Starts by randomly generating a number of acceleration readings various constraints

Integrates over acceleration readings to create "true path"

Applies gaussian noise to generated acceleration and gps readings

Result is a correct path and a noisy path to filter.

Stretch Goal is plot noisy, filtered, and true paths and calculate deviation of 
filtered path from truth
'''

MAXIMUM_JERK = .5
MAMIMUM_ACCEL = 10
MINIMUM_ACCEL = -10

DELTA_TIME = .005
END_TIME = 5
MAX_POINTS = END_TIME / DELTA_TIME

current_time = 0

def point_gen():
    last_point = random.randint(MINIMUM_ACCEL, MAMIMUM_ACCEL)
    num_points = 1
    yield last_point
    while num_points < MAX_POINTS:
        new_point = random.uniform(last_point - MAXIMUM_JERK, last_point + MAXIMUM_JERK)
        last_point = new_point
        yield new_point

accel_points_x = np.fromiter(point_gen(), float, MAX_POINTS)
accel_points_y = np.fromiter(point_gen(), float, MAX_POINTS)
accel_points_z = np.fromiter(point_gen(), float, MAX_POINTS)

print("X:", accel_points_x, "Y:", accel_points_y, "Z:", accel_points_z)
