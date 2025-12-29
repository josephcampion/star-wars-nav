

import numpy as np
import models.aerosonde_uav as mav
import models.aerodynamics as aero
from models.propulsion import EngineProperties as prop
from models.mass_props import MassProperties as mass
from models.vehicle import Vehicle
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

# TODO: Move examples like this to test_vehicle.py or test_dynamics.py
if __name__ == "__main__":

    aero_coeffs = aero.AerodynamicCoefficients([
        mav.C_L_0, mav.C_D_0, mav.C_m_0, \
        mav.C_L_alpha, mav.C_D_alpha, mav.C_m_alpha, \
        mav.C_L_q, mav.C_D_q, mav.C_m_q, \
        mav.C_L_delta_e, mav.C_D_delta_e, mav.C_m_delta_e, \
        mav.C_Y_0, mav.C_l_0, mav.C_n_0, \
        mav.C_Y_beta, mav.C_l_beta, mav.C_n_beta, \
        mav.C_Y_p, mav.C_l_p, mav.C_n_p, \
        mav.C_Y_r, mav.C_l_r, mav.C_n_r, \
        mav.C_Y_delta_a, mav.C_l_delta_a, mav.C_n_delta_a, \
        mav.C_Y_delta_r, mav.C_l_delta_r, mav.C_n_delta_r \
    ])

    engine_props = prop([mav.S_prop, \
        mav.k_motor, mav.k_T_p, mav.k_Omega, mav.C_prop])

    mass_props = mass(mav.m, mav.Jx, mav.Jy, mav.Jz, mav.Jxz)

    vehicle = Vehicle(aero_coeffs, engine_props, mass_props, mav.S, mav.c, mav.b, mav.rho)

    pn = 0.0
    pe = 0.0
    pd = 0.0
    u = 40.0
    v = 0.0
    w = 0.0
    phi = np.deg2rad(0.0)
    theta = np.deg2rad(3.0)
    psi = np.deg2rad(45.0)
    p = np.deg2rad(0.0)
    q = np.deg2rad(0.0)
    r = np.deg2rad(0.0)
    state = np.array([pn, pe, pd, u, v, w, phi, theta, psi, p, q, r])

    delta_e = 0.1
    delta_t = 1.0
    delta_a = 0.0
    delta_r = 0.0
    control = np.array([delta_e, delta_t, delta_a, delta_r])

    F_total, M_total = get_forces_and_moments(vehicle, state, control)

    print("F_total = ", F_total)
    print("M_total = ", M_total)

