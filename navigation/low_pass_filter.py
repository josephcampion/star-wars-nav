
import numpy as np

class LowPassFilter:
    def __init__(self, fc_hz, ts, y0=0.0):
        self._fc_hz = fc_hz
        self._ts = ts
        self._alpha = np.exp(-2*np.pi*fc_hz*ts)
        print("alpha = ", self._alpha)
        self._y = y0

    def update(self, u):
        self._y = self._alpha * self._y + (1 - self._alpha) * u
        return

    def get_filt_val(self):
        return self._y
