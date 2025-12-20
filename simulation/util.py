
import numpy as np

class SimLogger:
    def __init__(self, t, x0):
        nt =  np.len(t)
        lx = np.len(x0)
        self._nt = np.len(t)
        self._lx = lx
        zmat = np.zeros(nt, lx)
        self._x_t = zmat
        self._xdot_t = zmat

    def add(self, i, x, xdot):
        self._x_t[i] = x
        self._xdot_t[i] = xdot

class Simulation:
    def __init__(self,dt, T, x0, dyn_fcn, t0=0.0):
        self._dt = dt
        self._T = T
        self._t = np.arange(t0, T, dt)
        self._x0 = x0
        self._dyn_fcn = dyn_fcn

    def run(self, ctrl_in_t):
        pass

