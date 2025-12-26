
import numpy as np

class KinematicState:
    def __init__(self, init_conds=np.zeros(12)):
        self._pn = init_conds[0]
        self._pe = init_conds[1]
        self._pd = init_conds[2]
        self._u = init_conds[3]
        self._v = init_conds[4]
        self._w = init_conds[5]
        # TODO: Replace Euler angles with quaternions
        self._phi = init_conds[6]
        self._theta = init_conds[7]
        self._psi = init_conds[8]
        self._p = init_conds[9]
        self._q = init_conds[10]
        self._r = init_conds[11]

    def get_state(self):
        return np.array([
            self._pn,
            self._pe,
            self._pd,
            self._u,
            self._v,
            self._w,
            self._phi,
            self._theta,
            self._psi,
            self._p,
            self._q,
            self._r,
        ])
    
    def set_state(self, x_vec):
        self._pn = x_vec[0]
        self._pe = x_vec[1]
        self._pd = x_vec[2]
        self._u = x_vec[3]
        self._v = x_vec[4]
        self._w = x_vec[5]
        self._phi = x_vec[6]
        self._theta = x_vec[7]
        self._psi = x_vec[8]
        self._p = x_vec[9]
        self._q = x_vec[10]
        self._r = x_vec[11]

    # TODO: Output airspeed, course, vertical flight path angle

####################################################
# TODO: Make state derivatives given forces & moments
####################################################

class LinearKinematicState(KinematicState):
    def __init__(self, init_conds=np.zeros(12)):
        super().__init__(init_conds=init_conds)

    def get_lon_state(self):
        return np.array([
            self._u,
            self._w,
            self._q,
            self._theta,
        ])
    
    def get_lat_state(self):
        return np.array([
            self._v,
            self._p,
            self._phi,
            self._r,
        ])

class NonlinearKinematicState(KinematicState):
    def __init__(self, init_conds=np.zeros(12)):
        super().__init__(init_conds=init_conds)

    def solve_f_equals_ma(self, F, M):
        pass
