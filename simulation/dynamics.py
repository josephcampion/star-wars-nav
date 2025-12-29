

import numpy as np
from models.vehicle import Vehicle
import models.aerodynamics as aero
import simulation.rotations as rt

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

# end LinearDynamics4x4

def get_gravity_force(m, roll, pitch):

    GRAVITY_ACCELERATION = 9.81 # [m/s^2]
    
    DCM_ned2bod = rt.get_dcm_ned2bod(roll, pitch, 0.0)

    F_grav_ned = np.array([0.0, 0.0, m * GRAVITY_ACCELERATION])

    F_grav_bod = DCM_ned2bod @ F_grav_ned

    return F_grav_bod

# TODO: Move this to dynamics? And pass in 'vehicle' class to that?
def get_forces_and_moments(vehicle, state, control):
    m = vehicle._mass_props.get_mass()
    S = vehicle._S
    c = vehicle._c
    b = vehicle._b
    rho = vehicle._rho

    # p_ned = state[0:3]
    u, v, w = state[3:6]
    rpy = state[6:9]
    pqr = state[9:12]

    # TODO: Calculate alpha, beta, Va (ignoring wind for now).
    Va = aero.get_Va(u, v, w)
    alpha = aero.get_alpha(u, v, w)
    beta = aero.get_beta(u, v, w)

    delta_t = control[1]

    F_aero, M_aero = vehicle.get_aero_forces_and_moments(alpha, beta, Va, pqr, control, rho, S, c, b)
    F_prop, M_prop = vehicle.get_prop_force_and_moment(Va, delta_t)
    F_grav = get_gravity_force(m, rpy[0], rpy[1])

    return F_aero + F_prop + F_grav, M_aero + M_prop
