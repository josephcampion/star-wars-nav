

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

# TODO: Produce [fx, fy, fz]^T and [l, m, n]^T 
# given control inputs and ambient conditions.
class ForcesAndMoments:
    def __init__(self, aero_coeffs, mass_props, engine_props):
        pass

    def get_aero_forces_and_moments(self):
        pass

    def get_gravitational_force(self):
        pass

    def get_propulsion_forces(self):
        pass
