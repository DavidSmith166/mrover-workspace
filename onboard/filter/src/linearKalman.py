import numpy as np
from filterpy.kalman import KalmanFilter


class LinearKalman:
    # Class for linear kalman filtering

    def __init__(self, x_initial, P_initial, Q, R, dt):
        self.kf = KalmanFilter(dim_x=4, dim_z=6, dim_u=2)
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
        self.kf.H = np.array([[1., 1./60., 0., 0., 0.5*dt**2., 0.],
                         [0., 0., 0., 0., dt, 0.],
                         [0., 0., 1., 1./60., 0., 0.5*dt**2.],
                         [0., 0., 0., 0., 0., dt]])
        if np.isscalar(Q):
            self.kf.Q = Q_discrete_white_noise(dim=4, dt=dt, var=Q)
        else:
            self.kf.Q[:] = Q
        self.kf.R *= R
