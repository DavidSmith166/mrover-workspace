import numpy as np
# import random

'''
Goal is to simulate the sensor inputs one could expect to see for a given path

Starts by randomly generating a number of acceleration readings under various constraints

Integrates over acceleration readings to create "true path"

Applies normal distribution to generated acceleration and gps readings

Result is a correct path and a noisy path to filter.

Stretch Goal is plot noisy, filtered, and true paths and calculate deviation of
filtered path from truth
'''
# sudo apt-get install python3-tk needed to work
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.interpolate import interp1d
from scipy.misc import derivative
import matplotlib.pyplot as plt

x = np.linspace(0, 10, num=11, endpoint=True)
y = np.array([1, 3, 3, -2, 5, 6, 7, 6, 9, 3, 11])
f = interp1d(x, y)
f2 = interp1d(x, y, kind='cubic')

vel = [ derivative(f2, point, n=1) for point in x[1:-1] ]
acc = [ derivative(f2, point, n=2) for point in x[1:-1] ]
print(vel, acc)

xnew = np.linspace(0, 10, num=41, endpoint=True)
plt.plot(x, y, 'o', xnew, f(xnew), '-', xnew, f2(xnew), '--')
plt.legend(['data', 'linear', 'cubic'], loc='best')
plt.show()

# MAXIMUM_JERK = .5
# MAXIMUM_ACCEL = 5
# MINIMUM_ACCEL = -5

# DELTA_TIME = .005
# END_TIME = 5
# MAX_POINTS = np.int_(END_TIME / DELTA_TIME)
# NOISE_SCALE = 0.5


# def point_gen():
#     last_point = random.randint(MINIMUM_ACCEL, MAXIMUM_ACCEL)
#     num_points = 1
#     yield np.float64(last_point)
#     while num_points < MAX_POINTS:
#         new_point = random.uniform(max(last_point - MAXIMUM_JERK, MINIMUM_ACCEL),
#                                    min(last_point + MAXIMUM_JERK, MAXIMUM_ACCEL))
#         last_point = new_point
#         yield np.float64(new_point)


# def path_gen():

#     '''
#     ### Units are Deg/s
#     '''

#     while True:

#         accel_points_x = np.fromiter(point_gen(), np.float64, count=MAX_POINTS)
#         accel_points_y = np.fromiter(point_gen(), np.float64, count=MAX_POINTS)
#         accel_points_z = np.fromiter(point_gen(), np.float64, count=MAX_POINTS)

#         vel_points_x = np.trapz(accel_points_x, dx=DELTA_TIME)
#         vel_points_y = np.trapz(accel_points_y, dx=DELTA_TIME)
#         vel_points_z = np.trapz(accel_points_z, dx=DELTA_TIME)

#         gps_points_x = np.trapz(accel_points_x, dx=DELTA_TIME)
#         gps_points_y = np.trapz(accel_points_y, dx=DELTA_TIME)
#         gps_points_z = np.trapz(accel_points_z, dx=DELTA_TIME)

#         truth = {
#             "accel_x": accel_points_x,
#             "accel_y": accel_points_y,
#             "accel_z": accel_points_z,
#             "vel_x": vel_points_x,
#             "vel_y": vel_points_y,
#             "vel_z": vel_points_z,
#             "gps_x": gps_points_x,
#             "gps_y": gps_points_y,
#             "gps_z": gps_points_z
#         }

#         noisy_accel_points_x = np.random.normal(accel_points_x, NOISE_SCALE)
#         noisy_accel_points_y = np.random.normal(accel_points_y, NOISE_SCALE)
#         noisy_accel_points_z = np.random.normal(accel_points_z, NOISE_SCALE)

#         noisy_vel_points_x = np.random.normal(accel_points_x, NOISE_SCALE)
#         noisy_vel_points_y = np.random.normal(accel_points_y, NOISE_SCALE)
#         noisy_vel_points_z = np.random.normal(accel_points_z, NOISE_SCALE)

#         noisy_gps_points_x = np.random.normal(accel_points_x, NOISE_SCALE)
#         noisy_gps_points_y = np.random.normal(accel_points_y, NOISE_SCALE)
#         noisy_gps_points_z = np.random.normal(accel_points_z, NOISE_SCALE)

#         meas = {
#             "accel_x": noisy_accel_points_x,
#             "accel_y": noisy_accel_points_y,
#             "accel_z": noisy_accel_points_z,
#             "vel_x": noisy_vel_points_x,
#             "vel_y": noisy_vel_points_y,
#             "vel_z": noisy_vel_points_z,
#             "gps_x": noisy_gps_points_x,
#             "gps_y": noisy_gps_points_y,
#             "gps_z": noisy_gps_points_z
#         }

#         yield {"truth": truth, "meas": meas}
