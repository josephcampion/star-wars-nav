
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

####################################################
# TODO: Make state derivatives given forces & moments
####################################################


