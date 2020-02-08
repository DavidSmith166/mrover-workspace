import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from .conversions import meters2lat, meters2long


class LinearKalman:
    # Class for linear kalman filtering

    # KalmanFilter interface
    # ----------------------
    # __init__(dim_x, dim_z, dim_u)
    # state dimension = dim_x
    # input dimension = dim_z
    # control dimension = dim_u
    # x = dim_x zero vector
    # P = dim_x identity matrix
    # Q = dim_x identity matrix
    # B = None
    # F = dim_x identity matrix
    # H = dim_z X dim_x zero matrix
    # R = dim_z identity matrix
    # z = dim_z zero vector
    #
    # predict(u) (B, F, Q are optionally specified,
    #             we're using the initial matrices every time)
    # control variable vector = u
    #
    # update(z) (R, H are optionally specified,
    #            we're using the initial matrices every time)
    # measurement vector = z
    # --------------------------------
    # Q_discrete_white_noise interface
    # --------------------------------
    # Q_discrete_white_noise(dim, dt, var, block_size)
    # number of derivatives in state = dim
    # time step = dt
    # variance = var
    # number of dimensions in state = block_size

    def __init__(self, x_initial, P_initial, Q, R, dt):
        # construct the Kalman Filter
        self.kf = KalmanFilter(dim_x=4, dim_z=4, dim_u=2)
        x_initial = x_initial.asLKFInput()
        self.kf.x = np.array(x_initial)

        # convert P_initial from meters to degrees
        P_initial[:2] = meters2lat(P_initial[:2])
        P_initial[2:] = meters2long(P_initial[2:], x_initial[0])
        self.kf.P[:] = np.diag(P_initial)

        # convert R from meters to degrees
        R[:2] = meters2lat(R[:2])
        R[2:] = meters2long(R[2:], x_initial[0])
        self.kf.R[:] = np.diag(R)

        self.kf.F = np.array([[1., dt, 0., 0.],
                             [0., 1., 0., 0.],
                             [0., 0., 1., dt],
                             [0., 0., 0., 1.]])

        self.kf.B = np.array([[0.5*dt**2., 0.],
                             [dt, 0.],
                             [0., 0.5*dt**2.],
                             [0., dt]])

        self.kf.H = np.eye(4)
        # self.kf.H = np.diag([1, 0, 1, 0])

        # calculate process noise
        Q_lat = Q_discrete_white_noise(dim=2, dt=dt,
                                       var=meters2lat(Q),
                                       block_size=1)
        Q_long = Q_discrete_white_noise(dim=2, dt=dt,
                                        var=meters2long(Q, x_initial[0]),
                                        block_size=1)
        self.kf.Q = block_diag(Q_lat, Q_long)

    def run(self, gps, imu, state_estimate):
        # predicts forward given sensors
        # returns new state

        measured_pos = gps.asDecimal()

        measured_vel = gps.absolutifyVel(imu.bearing_degs)
        measured_vel.north = meters2lat(measured_vel.north)
        measured_vel.east = meters2long(measured_vel.east, measured_pos.lat_deg)

        measured_accel = imu.absolutifyAccel(imu.bearing_degs, imu.pitch_degs)
        measured_accel.north = meters2lat(measured_accel.north)
        measured_accel.east = meters2long(measured_accel.east, measured_pos.lat_deg)

        u = [measured_accel.north, measured_accel.east]

        z = [measured_pos.lat_deg, measured_vel.north,
             measured_pos.long_deg, measured_vel.east]

        self.kf.predict(np.array(u))
        self.kf.update(np.array(z))

        state_estimate.updateFromLKF(self.kf.x, imu.bearing_degs)
