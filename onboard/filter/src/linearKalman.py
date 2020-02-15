import numpy as np
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from .conversions import meters2lat, meters2long
from numpy import dot, zeros, eye, isscalar
from filterpy.common import reshape_z


class LinearKalmanFilter:
    # class for linear kalman filtering
    # Credit goes to Roger Labbe

    """
    Parameters
    ----------
    dim_x : int
        Number of state variables for the Kalman filter. For example, if
        you are tracking the position and velocity of an object in two
        dimensions, dim_x would be 4.
        This is used to set the default size of P, Q, and u
    dim_z : int
        Number of of measurement inputs. For example, if the sensor
        provides you with position in (x,y), dim_z would be 2.
    dim_u : int (optional)
        size of the control input, if it is being used.
        Default value of 0 indicates it is not used.
        off.
    Attributes
    ----------
    x : numpy.array(dim_x, 1)
        Current state estimate. Any call to update() or predict() updates
        this variable.
    P : numpy.array(dim_x, dim_x)
        Current state covariance matrix. Any call to update() or predict()
        updates this variable.
    z : numpy.array
        Last measurement used in update(). Read only.
    R : numpy.array(dim_z, dim_z)
        Measurement noise matrix
    Q : numpy.array(dim_x, dim_x)
        Process noise matrix
    F : numpy.array()
        State Transition matrix
    H : numpy.array(dim_z, dim_x)
        Measurement function
    y : numpy.array
        Residual of the update step. Read only.
    K : numpy.array(dim_x, dim_z)
        Kalman gain of the update step. Read only.
    S :  numpy.array
        System uncertainty (P projected to measurement space). Read only.
    SI :  numpy.array
        Inverse system uncertainty. Read only.
    inv : function, default numpy.linalg.inv
        If you prefer another inverse function, such as the Moore-Penrose
        pseudo inverse, set it to that instead: kf.inv = np.linalg.pinv
    """

    def __init__(self, dim_x, dim_z, dim_u=0):
        if dim_x < 1:
            raise ValueError('dim_x must be 1 or greater')
        if dim_z < 1:
            raise ValueError('dim_z must be 1 or greater')
        if dim_u < 0:
            raise ValueError('dim_u must be 0 or greater')

        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = zeros((dim_x, 1))         # state
        self.P = eye(dim_x)                # uncertainty covariance
        self.Q = eye(dim_x)                # process uncertainty
        self.B = None                      # control transition matrix
        self.F = eye(dim_x)                # state transition matrix
        self.H = zeros((dim_z, dim_x))     # Measurement function
        self.R = eye(dim_z)                # state uncertainty
        self.M = np.zeros((dim_z, dim_z))  # process-measurement cross correlation
        self.z = np.array([[None]*self.dim_z]).T

        # gain and residual are computed during the innovation step. We
        # save them so that in case you want to inspect them for various
        # purposes
        self.K = np.zeros((dim_x, dim_z))   # kalman gain
        self.y = zeros((dim_z, 1))
        self.S = np.zeros((dim_z, dim_z))   # system uncertainty
        self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty

        # identity matrix. Do not alter this.
        self._I = np.eye(dim_x)

        self.inv = np.linalg.inv

    def construct(self, x_initial, P_initial, Q, R, dt):
        # construct the Kalman Filter
        x_initial = x_initial.asLKFInput()
        self.x = np.array(x_initial)

        # convert P_initial from meters to degrees
        P_initial[:2] = meters2lat(P_initial[:2])
        P_initial[2:] = meters2long(P_initial[2:], x_initial[0])
        self.P[:] = np.diag(P_initial)

        # convert R from meters to degrees
        R[:2] = meters2lat(R[:2])
        R[2:] = meters2long(R[2:], x_initial[0])
        self.R[:] = np.diag(R)

        self.F = np.array([[1., dt, 0., 0.],
                           [0., 1., 0., 0.],
                           [0., 0., 1., dt],
                           [0., 0., 0., 1.]])

        self.B = np.array([[0.5*dt**2., 0.],
                           [dt, 0.],
                           [0., 0.5*dt**2.],
                           [0., dt]])

        self.H = np.eye(4)

        # calculate process noise
        Q_lat = Q_discrete_white_noise(dim=2, dt=dt,
                                       var=meters2lat(Q),
                                       block_size=1)
        Q_long = Q_discrete_white_noise(dim=2, dt=dt,
                                        var=meters2long(Q, x_initial[0]),
                                        block_size=1)
        self.Q = block_diag(Q_lat, Q_long)

    def predict(self, u=None, B=None, F=None, Q=None):
        """
        Predict next state (prior) using the Kalman filter state propagation
        equations.
        Parameters
        ----------
        u : np.array
            Optional control vector. If not `None`, it is multiplied by B
            to create the control input into the system.
        B : np.array(dim_x, dim_z), or None
            Optional control transition matrix; a value of None
            will cause the filter to use `self.B`.
        F : np.array(dim_x, dim_x), or None
            Optional state transition matrix; a value of None
            will cause the filter to use `self.F`.
        Q : np.array(dim_x, dim_x), scalar, or None
            Optional process noise matrix; a value of None will cause the
            filter to use `self.Q`.
        """

        if B is None:
            B = self.B
        if F is None:
            F = self.F
        if Q is None:
            Q = self.Q
        elif isscalar(Q):
            Q = eye(self.dim_x) * Q

        # x = Fx + Bu
        if B is not None and u is not None:
            self.x = dot(F, self.x) + dot(B, u)
        else:
            self.x = dot(F, self.x)

        # P = FPF' + Q
        self.P = dot(dot(F, self.P), F.T) + Q

    def update(self, z, R=None, H=None):
        """
        Add a new measurement (z) to the Kalman filter.
        If z is None, nothing is computed. However, x_post and P_post are
        updated with the prior (x_prior, P_prior), and self.z is set to None.
        Parameters
        ----------
        z : (dim_z, 1): array_like
            measurement for this update. z can be a scalar if dim_z is 1,
            otherwise it must be convertible to a column vector.
        R : np.array, scalar, or None
            Optionally provide R to override the measurement noise for this
            one call, otherwise  self.R will be used.
        H : np.array, or None
            Optionally provide H to override the measurement function for this
            one call, otherwise self.H will be used.
        """

        if z is None:
            self.z = np.array([[None]*self.dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            self.y = zeros((self.dim_z, 1))
            return

        z = reshape_z(z, self.dim_z, self.x.ndim)

        if R is None:
            R = self.R
        elif isscalar(R):
            R = eye(self.dim_z) * R

        if H is None:
            H = self.H

        # y = z - Hx
        # error (residual) between measurement and prediction
        self.y = z - dot(H, self.x)

        # common subexpression for speed
        PHT = dot(self.P, H.T)

        # S = HPH' + R
        # project system uncertainty into measurement space
        self.S = dot(H, PHT) + R
        self.SI = self.inv(self.S)
        # K = PH'inv(S)
        # map system uncertainty into kalman gain
        self.K = dot(PHT, self.SI)

        # x = x + Ky
        # predict new x with residual scaled by the kalman gain
        self.x = self.x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK'
        # This is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.

        I_KH = self._I - dot(self.K, H)
        self.P = dot(dot(I_KH, self.P), I_KH.T) + dot(dot(self.K, R), self.K.T)

    def run(self, gps, imu, state_estimate):
        # predicts forward given sensors
        # returns new state

        measured_pos = gps.pos.asDecimal()

        measured_vel = gps.vel.absolutify(imu.bearing.bearing_degs)
        measured_vel.north = meters2lat(measured_vel.north)
        measured_vel.east = meters2long(measured_vel.east, measured_pos.lat_deg)

        measured_accel = imu.accel.absolutify(imu.bearing.bearing_degs, imu.pitch_degs)
        measured_accel.north = meters2lat(measured_accel.north)
        measured_accel.east = meters2long(measured_accel.east, measured_pos.lat_deg)

        u = [measured_accel.north, measured_accel.east]

        z = [measured_pos.lat_deg, measured_vel.north,
             measured_pos.long_deg, measured_vel.east]

        self.predict(np.array(u))
        self.update(np.array(z))

        state_estimate.updateFromLKF(self.x, imu.bearing.bearing_degs)
    
    def runFromGps(self, gps, state_estimate):
        pass
    
    def runFromImu(self, imu, state_estimate):
        pass

    def runFromGpsImu(self, gps, imu, state_estimate):
        pass