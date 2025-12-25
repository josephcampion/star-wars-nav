
import numpy as np

class KinematicState:
    # [pn, pe, pd, phi, theta, psi, u, v, w, p, q, r]
    # [0,  1,  2,  3,   4,     5,   6, 7, 8, 9, 10,11]
    def __init__(self, init_conds=np.zeros(12)):
        self._state = init_conds

    def get_state(self):
        return self._state
    
    def set_state(self, X):
        self._state = X
    
    def get_lon_state(self):
        return np.array([
            self._state[6],  # u
            self._state[8],  # w
            self._state[10], # q
            self._state[4],  # theta
        ])
    
    def get_lat_state(self):
        return np.array([
            self._state[7],  # v
            self._state[9],  # p
            self._state[3],  # phi
            self._state[11], # r
        ])

    # TODO: Output airspeed, course, vertical flight path angle

def cross_mat(v):
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],  0.0,    -v[0]],
        [-v[1], v[0],   0.0]
    ])

def inertia_mat(Jx, Jy, Jz, Jxz):
    return np.array([
        [Jx,    0.0,    -Jxz],
        [0.0,   Jy,     0.0],
        [-Jxz,  0.0,    Jz]
    ])

def inertia_mat_inv(Jx, Jy, Jz, Jxz):

    gamma = Jz*Jz - Jxz**2

    return np.array([
        [Jz/gamma,  0.0,    Jxz/gamma],
        [0.0,       1/Jy,   0.0],
        [Jxz/gamma, 0.0,    Jx/gamma]
    ])

class InputAxisError(Exception):
    pass

def rot(angle_rad, axis):

    ca = np.cos(angle_rad)
    sa = np.sin(angle_rad)

    if axis == 0: # x
        return np.array([
            [1.0,   0.0,    0.0],
            [0.0,   ca,     sa],
            [0.0,   -sa,    ca]
        ])
    elif axis == 1: # y
        return np.array([
            [ca,    0.0,    -sa],
            [0.0,   1.0,    0.0],
            [sa,    0.0,    ca]
        ])
    elif axis == 2: # z
        return np.array([
            [ca,    sa,     0.0],
            [-sa,   ca,     0.0],
            [0.0,   0.0,    1.0]
        ])
    else:
        raise InputAxisError("Input axis must be 0, 1, or 2 (for x, y, or z).")

# NOTE: Angles are in radians (since np.sin/cos are too)
def get_dcm_ned2bod(roll, pitch, yaw):
    return rot(roll, 0) @ rot(pitch, 1) @ rot(yaw, 2)


