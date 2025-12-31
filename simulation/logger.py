
import numpy as np

class SimulationLogger:
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

    # TODO: Log AirDataState (when implemented)

    # TODO: Log ControlInputs

    # TODO: Log ForcesAndMoments

    # TODO: Log VehicleState

    # TODO: Log VehicleStateDot

