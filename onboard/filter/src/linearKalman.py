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
    # --------------------------------
    # Q_discrete_white_noise interface
    # --------------------------------
    # Q_discrete_white_noise(dim, dt, var, block_size)
    # number of derivatives in state = dim
    # time step = dt
    # variance = var
    # number of dimensions in state = block_size

    def __init__(self, x_initial, P_initial, Q, R, dt):
        # Construct the Kalman Filter
        self.kf = KalmanFilter(dim_x=4, dim_z=4, dim_u=2)
        self.kf.x = np.array(x_initial)
        self.kf.P[:] = np.diag(P_initial)
        self.kf.F = np.array([[1., dt, 0., 0.],
                             [0., 1., 0., 0.],
                             [0., 0., 1., dt],
                             [0., 0., 0., 1.]])
        self.kf.B = np.array([[0.5*dt**2., 0.],
                             [dt, 0.],
                             [0., 0.5*dt**2.],
                             [0., dt]])
        self.kf.H = np.eye(4)
        if np.isscalar(Q):
            self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q, block_size=2)
        else:
            self.kf.Q[:] = Q
        self.kf.R *= R

    def run(self, u, z):
        # Predicts forward using control u and updates with measurement z
        # Returns new state
        self.kf.predict(np.array(u))
        self.kf.update(np.array(z))

        return self.kf.x
