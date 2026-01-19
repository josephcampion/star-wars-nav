
import numpy as np
import models.aerosonde_uav as mav

import models.aerodynamics as aero
from models.propulsion import EngineProperties as prop
from models.mass_props import MassProperties as mass

from simulation.kinematics import NonlinearKinematicState as kin

class Vehicle:
    def __init__(self, aero_coeffs, engine_props, mass_props, S, c, b, rho):
        self._aero_coeffs = aero_coeffs
        self._engine_props = engine_props
        self._mass_props = mass_props
        self._S = S
        self._c = c
        self._b = b
        self._rho = rho
    
    def get_aero_forces_and_moments(self, alpha, beta, Va, pqr, control, rho, S, c, b):
        return self._aero_coeffs.get_aero_forces_and_moments(alpha, beta, Va, pqr, control, rho, S, c, b)

    def get_prop_force_and_moment(self, Va, delta_t):
        return self._engine_props.get_prop_force_and_moment(Va, delta_t)

    def get_mass_props(self):
        return self._mass_props

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

