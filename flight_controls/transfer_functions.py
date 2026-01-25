

import numpy as np
import control as ct # 3rd party
import models.aerosonde_uav as mav
from models.aerodynamics import AerodynamicCoefficients as aero_coeffs
from flight_controls.trim import get_trim_conditions
from models.mass_props import InertiaTerms
from simulation.dynamics import GRAV_ACCEL_MPS2


####################################################
#   Get A/C Parameters & Flight + Trim Conditions
####################################################

aero_coeffs = aero_coeffs([
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

inertia_terms = InertiaTerms(mav.Jx, mav.Jy, mav.Jz, mav.Jxz)

gamma_vec = inertia_terms.get_gamma_vec()
gamma3 = gamma_vec[3]
gamma4 = gamma_vec[4]
gamma8 = gamma_vec[8]

C_p_0, C_p_beta, C_p_p, C_p_r, C_p_delta_a, C_p_delta_r, \
    C_r_0, C_r_beta, C_r_p, C_r_r, C_r_delta_a, C_r_delta_r = \
            aero_coeffs.get_p_and_r_derivatives(gamma3, gamma4, gamma8)

C_Y_beta = mav.C_Y_beta
C_Y_delta_r = mav.C_Y_delta_r

C_m_q = mav.C_m_q
C_m_alpha = mav.C_m_alpha
C_m_delta_e = mav.C_m_delta_e

C_D_0 = mav.C_D_0
C_D_alpha = mav.C_D_alpha
C_D_delta_e = mav.C_D_delta_e

C_prop = mav.C_prop
k_T_p = mav.k_T_p
S_prop = mav.S_prop

S = mav.S
b = mav.b
c = mav.c
m = mav.m
Jy = mav.Jy

#--------------------Flight Conditions--------------------#
Va = 30.0
# TODO: Make density a function of altitude
rho = mav.rho

g = GRAV_ACCEL_MPS2
q_inf = 0.5 * rho * Va**2

#--------------------Trim Conditions--------------------#
trim_vec = get_trim_conditions(Va)
alpha_trim = trim_vec[0]
beta_trim = trim_vec[1]
u_trim = trim_vec[2]
v_trim = trim_vec[3]
w_trim = trim_vec[4]
theta_trim = trim_vec[5]
p_trim = trim_vec[6]
q_trim = trim_vec[7]
r_trim = trim_vec[8]
delta_e_trim = trim_vec[9]
delta_t_trim = trim_vec[10]
delta_a_trim = trim_vec[11]
delta_r_trim = trim_vec[12]
Va_trim = Va

# Laplace Operator
s = ct.tf('s')

####################################################
        #   Lateral Transfer Functions
####################################################

#--------------------Roll TF-----------------------#
# This assumes small pitch: tan(theta) ~= 0 --> phi_dot ~= p
a_phi_1 = -1.0 * q_inf * S * b * C_p_p * b / (2 * Va)
a_phi_2 = q_inf * S * b * C_p_delta_a

#--------------------Delta Aileron to Roll Rate TF-----------------------#
tf_da_to_p = a_phi_1 / (s + a_phi_2)
tf_da_to_roll = a_phi_1 / (s * (s + a_phi_2))

#--------------------Course TF-----------------------#
# This assumes coordinated turns: chi = (g/Vg) * tan(phi)
tf_roll_to_course = (g / Va) / s

#--------------------Sideslip TF-----------------------#
# This assumes no winds and small sideslip: v = Va * sin(beta)

c1 = rho * Va * S / (2 * m)
a_beta_1 = -1 * c1 * C_Y_beta
a_beta_2 = c1 * C_Y_delta_r

tf_dr_to_beta = a_beta_2 / (s + a_beta_1)

# TODO: Make tf_rudder_to_ay_tf

####################################################
    #   Longitudinal Transfer Functions
####################################################

#--------------------Pitch TF-----------------------#
# This assumes small roll:
# theta_dot = q * cos(phi) - r*sin(phi) -> theta_dot ~= q
# --> theta_ddot ~= q_dot

c1 = rho * Va**2 * c * S / (2 * Jy)
a_theta_1 = -1 * c1 * C_m_q * c / (2 * Va)
a_theta_2 =  -1 * c1 * C_m_alpha
a_theta_3 = c1 * C_m_delta_e

tf_de_to_pitch = a_theta_3 / (s**2 + a_theta_1 * s + a_theta_2)

#--------------------Delta Elevator to Pitch Rate TF-----------------------#
tf_de_to_q = tf_de_to_pitch * s


#--------------------Altitude TF-----------------------#
# Assumptions: v~=0, w~=0, phi~=0, u~=Va, phi~=0, theta small.
# h_dot = u*sin(theta) - v*sin(phi)*cos(theta) - w*cos(phi)*cos(theta)
# --> h_dot ~= Va*theta
tf_theta_to_h = Va / s

#----------------Throttle to Airspeed TF-----------------------#
# Va = sqrt(u**2 + v**2 + w**2)
# --> Va_dot = (u*u_dot + v*v_dot + w*w_dot) / Va
# Assuming no winds:
# Va = u*cos(alpha)*cos(beta) + v*sin(beta) + w*sin(alpha)*cos(beta)
# (after trim assumptions and lots o' math...)
# Va_dot = -g*cos(theta_trim - alpha_trim)*theta_bar + K1 * Va_bar + K2 * delta_t_bar    
c1 = rho * Va_trim * S / m
c2 = rho * S_prop / m * C_prop
c3 = C_D_0 + C_D_alpha * alpha_trim + C_D_delta_e * delta_e_trim

a_Va_1 = c1 * c3 + c2 * Va_trim
a_Va_2 = c2 * k_T_p**2 * delta_t_trim

tf_throttle_to_airspeed = a_Va_2 / (s + a_Va_1)

#-----------------Pitch to Airspeed TF-----------------------#
c1 = rho * Va_trim * S / m
c2 = rho * S_prop / m * C_prop
c3 = C_D_0 + C_D_alpha * alpha_trim + C_D_delta_e * delta_e_trim

a_Va_1 = c1 * c3 + c2 * Va_trim
a_Va_3 = g * np.cos(theta_trim - alpha_trim)

tf_pitch_to_airspeed = -a_Va_3 / (s + a_Va_1)

if __name__ == "__main__":
    
    print("Lateral Transfer Functions:")
    print(f"tf_da_to_p = {tf_da_to_p}")
    print(f"tf_da_to_roll = {tf_da_to_roll}")
    print(f"tf_roll_to_course = {tf_roll_to_course}")
    print(f"tf_dr_to_beta = {tf_dr_to_beta}")
    print("--------------------------------")

    print("Longitudinal Transfer Functions:")
    print(f"tf_de_to_q = {tf_de_to_q}")
    print(f"tf_de_to_pitch = {tf_de_to_pitch}")
    print(f"tf_theta_to_h = {tf_theta_to_h}")
    print(f"tf_throttle_to_airspeed = {tf_throttle_to_airspeed}")
    print(f"tf_pitch_to_airspeed = {tf_pitch_to_airspeed}")
    print("--------------------------------")
