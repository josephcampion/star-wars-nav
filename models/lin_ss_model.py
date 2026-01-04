
import numpy as np
import models.aerosonde_uav as ac
from models.utils import InertiaTerms
from models.aerodynamics import AerodynamicCoefficients
from models.propulsion import EngineProperties
from models.mass_props import MassProperties


def make_lin_ss_model(mass_props, engine_props, aero_coeffs, trim_conds):

    # TODO: Unpack all the parameters from the vehicle model.
    m = mass_props.m
    Jx = mass_props.Jx
    Jy = mass_props.Jy
    Jz = mass_props.Jz
    Jxz = mass_props.Jxz
    S_prop = engine_props.S_prop
    k_motor = engine_props.k_motor
    k_T_p = engine_props.k_T_p
    k_Omega = engine_props.k_Omega
    e = engine_props.e
    C_prop = engine_props.C_prop
    M = engine_props.M
    eta = engine_props.eta
    C_L_0 = aero_coeffs.C_L_0
    C_D_0 = aero_coeffs.C_D_0
    C_m_0 = aero_coeffs.C_m_0
    C_L_alpha = aero_coeffs.C_L_alpha
    C_D_alpha = aero_coeffs.C_D_alpha
    C_m_alpha = aero_coeffs.C_m_alpha
    C_L_q = aero_coeffs.C_L_q
    C_D_q = aero_coeffs.C_D_q
    C_m_q = aero_coeffs.C_m_q
    C_L_delta_e = aero_coeffs.C_L_delta_e
    C_D_delta_e = aero_coeffs.C_D_delta_e
    C_m_delta_e = aero_coeffs.C_m_delta_e
    C_Y_0 = aero_coeffs.C_Y_0
    C_l_0 = aero_coeffs.C_l_0
    C_n_0 = aero_coeffs.C_n_0
    C_Y_beta = aero_coeffs.C_Y_beta
    C_l_beta = aero_coeffs.C_l_beta
    C_n_beta = aero_coeffs.C_n_beta
    C_Y_p = aero_coeffs.C_Y_p
    C_l_p = aero_coeffs.C_l_p
    C_n_p = aero_coeffs.C_n_p
    C_Y_r = aero_coeffs.C_Y_r
    C_l_r = aero_coeffs.C_l_r
    C_n_r = aero_coeffs.C_n_r
    C_Y_delta_a = aero_coeffs.C_Y_delta_a
    C_l_delta_a = aero_coeffs.C_l_delta_a
    C_n_delta_a = aero_coeffs.C_n_delta_a
    C_Y_delta_r = aero_coeffs.C_Y_delta_r
    C_l_delta_r = aero_coeffs.C_l_delta_r
    C_n_delta_r = aero_coeffs.C_n_delta_r
    alpha_0 = trim_conds.alpha_0
    beta_0 = trim_conds.beta_0
    Va = trim_conds.Va
    p_0 = trim_conds.p_0
    q_0 = trim_conds.q_0
    r_0 = trim_conds.r_0

    pass


class LinearStateSpaceModel:
    def __init__(self, mass_props, engine_props, aero_coeffs, trim_conds):
        self._mass_props = mass_props
        self._engine_props = engine_props
        self._aero_coeffs = aero_coeffs
        self._trim_conds = trim_conds
        

if __name__ == "__main__":

    mass = MassProperties(ac.m, [ac.Jx, ac.Jy, ac.Jz, ac.Jxz])

    engine_props = EngineProperties([
        ac.S_prop, ac.k_motor, ac.k_T_p, ac.k_Omega, ac.e, \
        ac.C_prop, ac.M, ac.eta
    ])

    aero_coeffs = AerodynamicCoefficients([
        ac.C_L_0, ac.C_D_0, ac.C_m_0, \
        ac.C_L_alpha, ac.C_D_alpha, ac.C_m_alpha, \
        ac.C_L_q, ac.C_D_q, ac.C_m_q, \
        ac.C_L_delta_e, ac.C_D_delta_e, ac.C_m_delta_e, \
        ac.C_Y_0, ac.C_l_0, ac.C_n_0, \
        ac.C_Y_beta, ac.C_l_beta, ac.C_n_beta, \
        ac.C_Y_p, ac.C_l_p, ac.C_n_p, \
        ac.C_Y_r, ac.C_l_r, ac.C_n_r, \
        ac.C_Y_delta_a, ac.C_l_delta_a, ac.C_n_delta_a, \
        ac.C_Y_delta_r, ac.C_l_delta_r, ac.C_n_delta_r
    ])


    # print(dir(mass_props))
    # print(dir(engine_props))
    # print(dir(aero_coeffs))
