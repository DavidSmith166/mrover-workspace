import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


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


    def __init__(self, x_initial, P_initial, Q, R, dt):
        # Builds the Kalman Filter
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=2)
        self.kf.x = np.array(x_inital)
        if np.isscalar(P_initial):
            self.kf.P *= P_initial
        else:
            self.kf.P[:] = P_inital
        self.kf.F = np.array([[1., dt, 0., 0.],
                             [0., 1., 0., 0.],
                             [0., 0., 1., dt],
                             [0., 0., 0., 1.]])
        self.kf.B = np.array([[0.5*dt**2., 0.],
                             [dt, 0.],
                             [0., 0.5*dt**2.],
                             [0., dt]])
        self.kf.H = np.array([[1., 0., 0., 0.],
                              [0., 0., 1., 0.]])
        if np.isscalar(Q):
            self.kf.Q = Q_discrete_white_noise(dim=4, dt=dt, var=Q)
        else:
            self.kf.Q[:] = Q
        self.kf.R *= R

    def run(self, u, z):
        # Predicts forward using control u and updates with measurement z
        # Returns new state
        self.kf.predict(u)
        self.kf.update(z)

        return self.kf.x
