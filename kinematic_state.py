
import numpy as np

class KinematicState:
    # [pn, pe, pd, phi, theta, psi, u, v, w, p, q, r]
    def __init__(self, init_conds=np.zeros(12)):
        self._state = init_conds

    def get_state(self):
        return self._state

    # TODO: Output airspeed, course, vertical flight path angle
