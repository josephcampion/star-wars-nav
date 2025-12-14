

# import numpy as np

class LinearDynamics4x4:
    # xdot = A * x + B * u
    # y = C * x + D * u
    def __init__(self, A, B, C, D):
        self._A = A
        self._B = B
        self._C = C
        self._D = D

    def propagate_state(self, x, u):

        xdot = self._A @ x + self._B @ u
        y = self._C @ x + self._D @ u
        
        return xdot # , y
