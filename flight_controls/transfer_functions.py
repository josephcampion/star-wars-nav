

import numpy as np
import control as ct # 3rd party
import models.aerosonde_uav as uav
from models.aerodynamics import AerodynamicCoefficients as aero_coeffs
from models.mass_props import InertiaTerms


####################################################
    #   Get C_p_p and C_p_delta_a
####################################################

aero_coeffs = aero_coeffs([
    uav.C_L_0, uav.C_D_0, uav.C_m_0, \
    uav.C_L_alpha, uav.C_D_alpha, uav.C_m_alpha, \
    uav.C_L_q, uav.C_D_q, uav.C_m_q, \
    uav.C_L_delta_e, uav.C_D_delta_e, uav.C_m_delta_e, \
    uav.C_Y_0, uav.C_l_0, uav.C_n_0, \
    uav.C_Y_beta, uav.C_l_beta, uav.C_n_beta, \
    uav.C_Y_p, uav.C_l_p, uav.C_n_p, \
    uav.C_Y_r, uav.C_l_r, uav.C_n_r, \
    uav.C_Y_delta_a, uav.C_l_delta_a, uav.C_n_delta_a, \
    uav.C_Y_delta_r, uav.C_l_delta_r, uav.C_n_delta_r \
])

inertia_terms = InertiaTerms(uav.Jx, uav.Jy, uav.Jz, uav.Jxz)

gamma_vec = inertia_terms.get_gamma_vec()
gamma3 = gamma_vec[3]
gamma4 = gamma_vec[4]
gamma8 = gamma_vec[8]

C_p_0, C_p_beta, C_p_p, C_p_r, C_p_delta_a, C_p_delta_r, \
    C_r_0, C_r_beta, C_r_p, C_r_r, C_r_delta_a, C_r_delta_r = \
            aero_coeffs.get_p_and_r_derivatives(gamma3, gamma4, gamma8)

####################################################
    #   Get Aileron to Roll Transfer Function
####################################################

def get_delta_a_to_roll_tf(rho, Va, S, b, C_p_p, C_p_delta_a):

    q_inf = 0.5 * rho * Va**2

    a_phi_1 = q_inf * S * b * C_p_p * b / (2 * Va)
    # print("a_phi_1 = ", a_phi_1)
    a_phi_2 = q_inf * S * b * C_p_delta_a
    # print("a_phi_2 = ", a_phi_2)

    s = ct.tf('s')

    return a_phi_1 / (s * (s + a_phi_2))

tf_roll = get_delta_a_to_roll_tf(uav.rho, 30.0, uav.S, uav.b, C_p_p, C_p_delta_a)

print(tf_roll)