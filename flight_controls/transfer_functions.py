

import numpy as np
import control as ct # 3rd party
import models.aerosonde_uav as uav
from models.aerodynamics import AerodynamicCoefficients as aero_coeffs
from models.lin_ss_model import Va_trim
from models.mass_props import InertiaTerms
from simulation.dynamics import GRAV_ACCEL_MPS2


####################################################
        #   Lateral Transfer Functions
####################################################

# This assumes small pitch: tan(theta) ~= 0 --> phi_dot ~= p
def get_delta_a_to_roll_tf(rho, Va, S, b, C_p_p, C_p_delta_a):

    q_inf = 0.5 * rho * Va**2

    a_phi_1 = -1.0 * q_inf * S * b * C_p_p * b / (2 * Va)
    # print("a_phi_1 = ", a_phi_1)
    a_phi_2 = q_inf * S * b * C_p_delta_a
    # print("a_phi_2 = ", a_phi_2)

    s = ct.tf('s')

    return a_phi_1 / (s * (s + a_phi_2))

# This assumes coordinated turns: chi = (g/Vg) * tan(phi)
def get_roll_to_course_tf(Va):

    g = GRAV_ACCEL_MPS2

    s = ct.tf('s')

    return (g / Va) / s

# This assumes no winds and small sideslip: v = Va * sin(beta)
def get_rudder_to_sideslip_tf(rho, Va, S, b, m, C_Y_beta, C_Y_delta_r):

    c1 = rho * Va * S / (2 * m)

    a_beta_1 = -1 * c1 * C_Y_beta

    a_beta_2 = c1 * C_Y_delta_r

    s = ct.tf('s')

    return a_beta_2 / (s + a_beta_1)

def get_rudder_to_ay_tf():
    pass

####################################################
    #   TODO: Longitudinal Transfer Functions
####################################################

# This assumes small roll:
# theta_dot = q * cos(phi) - r*sin(phi) -> theta_dot ~= q
# --> theta_ddot ~= q_dot
def get_delta_e_to_pitch_tf(rho, Va, S, c, Jy, C_m_q, C_m_alpha, C_m_delta_e):

    c1 = rho * Va**2 * c * S / (2 * Jy)
    
    a_theta_1 = -1 * c1 * C_m_q * c / (2 * Va)

    a_theta_2 =  -1 * c1 * C_m_alpha

    a_theta_3 = c1 * C_m_delta_e

    s = ct.tf('s')

    return a_theta_3 / (s**2 + a_theta_1 * s + a_theta_2)

# Assumptions: v~=0, w~=0, phi~=0, u~=Va, phi~=0, theta small.
# h_dot = u*sin(theta) - v*sin(phi)*cos(theta) - w*cos(phi)*cos(theta)
# --> h_dot ~= Va*theta
def get_pitch_to_altitude_tf(Va):

    s = ct.tf('s')
    
    return Va / s

# Va = sqrt(u**2 + v**2 + w**2)
# --> Va_dot = (u*u_dot + v*v_dot + w*w_dot) / Va
# Assuming no winds:
# Va = u*cos(alpha)*cos(beta) + v*sin(beta) + w*sin(alpha)*cos(beta)
# (after trim assumptions and lots o' math...)
# Va_dot = -g*cos(theta_trim - alpha_trim)*theta_bar + K1 * Va_bar + K2 * delta_t_bar
def get_throttle_to_airspeed_tf():
    
    c1 = rho * Va_trim * S / m
    c2 = rho * S_prop / m * C_prop
    c3 = C_D_0 + C_D_alpha * alpha_trim + C_D_delta_e * delta_e_trim

    a_Va_1 = c1 * c3 + c2 * Va_trim

    a_Va_2 = c2 * k_T_p**2 * delta_t_trim

    s = ct.tf('s')

    return a_Va_2 / (s + a_Va_1)

def get_pitch_to_airspeed_tf():
        
    c1 = rho * Va_trim * S / m
    c2 = rho * S_prop / m * C_prop
    c3 = C_D_0 + C_D_alpha * alpha_trim + C_D_delta_e * delta_e_trim

    a_Va_1 = c1 * c3 + c2 * Va_trim

    a_Va_3 = g * cos(theta_trim - alpha_trim)

    s = ct.tf('s')

    return -a_Va_3 / (s + a_Va_1)

####################################################
            #   Make all TFs
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

# Flight Conditions
Va = 30.0
# TODO: Make density a function of altitude
rho = uav.rho
S = uav.S
b = uav.b
c = uav.c
m = uav.m
Jy = uav.Jy

C_Y_beta = uav.C_Y_beta
C_Y_delta_r = uav.C_Y_delta_r

tf_da_to_roll = get_delta_a_to_roll_tf(rho, Va, S, b, C_p_p, C_p_delta_a)
tf_roll_to_course =  get_roll_to_course_tf(Va)
tf_rudder_to_beta = get_rudder_to_sideslip_tf(rho, Va, S, b, m, C_Y_beta, C_Y_delta_r)

C_m_q = uav.C_m_q
C_m_alpha = uav.C_m_alpha
C_m_delta_e = uav.C_m_delta_e

tf_de_to_pitch = get_delta_e_to_pitch_tf(rho, Va, S, c, Jy, C_m_q, C_m_alpha, C_m_delta_e)
tf_theta_to_h = get_pitch_to_altitude_tf(Va)

if __name__ == "__main__":
    
    print("Lateral Transfer Functions:")
    print(f"tf_da_to_roll = {tf_da_to_roll}")
    print(f"tf_roll_to_course = {tf_roll_to_course}")
    print(f"tf_rudder_to_beta = {tf_rudder_to_beta}")
    print("--------------------------------")

    print("Longitudinal Transfer Functions:")
    print(f"tf_de_to_pitch = {tf_de_to_pitch}")
    print(f"tf_theta_to_h = {tf_theta_to_h}")
    print("--------------------------------")
