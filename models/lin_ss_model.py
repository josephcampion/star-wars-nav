
import numpy as np
import models.aerosonde_uav as ac
from models.aerodynamics import AerodynamicCoefficients
from models.propulsion import EngineProperties
from models.mass_props import MassProperties
import flight_controls.trim as trim

AIR_DENSITY_KGM3 = 1.2682 # [kg/m^3] # TODO: Make this a function of altitude?


mass_props = MassProperties(ac.m, ac.Jx, ac.Jy, ac.Jz, ac.Jxz)

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

# TODO: Figure out efficient flight condition (minimize drag).
Va_trim = 30.0 # [m/s]
trim_vec = trim.get_trim_conditions(Va_trim)
print("trim_vec = ", trim_vec)
alpha_trim = trim_vec[0]
beta_trim = trim_vec[1]
u_trim = trim_vec[2]
v_trim = trim_vec[3]
w_trim = trim_vec[4]
theta_trim = trim_vec[5]
p_trim = trim_vec[6]
q_trim = trim_vec[7]
r_trim = trim_vec[8]
delta_a_trim = trim_vec[9]
delta_r_trim = trim_vec[10]
delta_e_trim = trim_vec[11]
delta_t_trim = trim_vec[12]

# TODO: Store all the parameters in a single struct.
rho = AIR_DENSITY_KGM3
m = mass_props.get_mass()
J_mat = mass_props.get_inertia_matrix()
Jx = J_mat[0,0]
Jy = J_mat[1,1]
Jz = J_mat[2,2]
Jxz = J_mat[0,2]
S = ac.S
b = ac.b
c = ac.c
g = 9.81 # [m/s^2]
S_prop = ac.S_prop
k_motor = ac.k_motor
k_T_p = ac.k_T_p
k_Omega = ac.k_Omega
e = ac.e
C_prop = ac.C_prop
M = ac.M
eta = ac.eta
C_L_0 = ac.C_L_0
C_D_0 = ac.C_D_0
C_m_0 = ac.C_m_0
C_L_alpha = ac.C_L_alpha
C_D_alpha = ac.C_D_alpha
C_m_alpha = ac.C_m_alpha
C_L_q = ac.C_L_q
C_D_q = ac.C_D_q
C_m_q = ac.C_m_q
C_L_delta_e = ac.C_L_delta_e
C_D_delta_e = ac.C_D_delta_e
C_m_delta_e = ac.C_m_delta_e
C_Y_0 = ac.C_Y_0
C_l_0 = ac.C_l_0
C_n_0 = ac.C_n_0
C_Y_beta = ac.C_Y_beta
C_l_beta = ac.C_l_beta
C_n_beta = ac.C_n_beta
C_Y_p = ac.C_Y_p
C_l_p = ac.C_l_p
C_n_p = ac.C_n_p
C_Y_r = ac.C_Y_r
C_l_r = ac.C_l_r
C_n_r = ac.C_n_r
C_Y_delta_a = ac.C_Y_delta_a
C_l_delta_a = ac.C_l_delta_a
C_n_delta_a = ac.C_n_delta_a
C_Y_delta_r = ac.C_Y_delta_r
C_l_delta_r = ac.C_l_delta_r
C_n_delta_r = ac.C_n_delta_r

# TODO: Get gammaX from mass props to convert 
# aerodynamic coefficients into p and r derivatives.
gamma_vec = mass_props.get_gamma_vec()
gamma1 = gamma_vec[1]
gamma2 = gamma_vec[2]
gamma3 = gamma_vec[3]
gamma4 = gamma_vec[4]
# gamma5 = gamma_vec[5]
# gamma6 = gamma_vec[6]
gamma7 = gamma_vec[7]
gamma8 = gamma_vec[8]

(C_p_0, C_p_beta, C_p_p, C_p_r, C_p_delta_a, C_p_delta_r, \
    C_r_0, C_r_beta, C_r_p, C_r_r, C_r_delta_a, C_r_delta_r) \
        = aero_coeffs.get_p_and_r_derivatives(gamma3, gamma4, gamma8)

c1 = rho * S * v_trim / (4 * m * Va_trim)
c2 = rho * S * v_trim / m
c3 = rho * S * C_Y_beta / (2 * m)
c4 = C_Y_p * p_trim + C_Y_r * r_trim
c5 = C_Y_0 + C_Y_beta * beta_trim + C_Y_delta_a * delta_a_trim + C_Y_delta_r * delta_r_trim
c6 = np.sqrt(u_trim**2 + w_trim**2)
c7 = rho * Va_trim * S * b / (4 * m)
c8 = rho * S * b**2 * v_trim / (4 * Va_trim)
c9 = C_p_p * p_trim + C_p_r * r_trim
c10 = C_p_0 + C_p_beta * beta_trim + C_p_delta_a * delta_a_trim + C_p_delta_r * delta_r_trim
c11 = rho * S * b * v_trim
c12 = rho * S * b * C_p_beta / 2
c13 = rho * Va_trim * S * b**2 / 4
c14 = C_r_p * p_trim + C_r_r * r_trim
c15 = C_r_0 + C_r_beta * beta_trim + C_r_delta_a * delta_a_trim + C_r_delta_r * delta_r_trim
c16 = rho * S * b * C_r_beta / 2
c17 = rho * Va_trim**2 * S / (2 * m)

Y_v = c1 * c4 + c2 * c5 + c3 * c6
Y_p = w_trim + c7 * C_Y_p
Y_r = -u_trim + c7 * C_Y_r

Y_delta_a = c17 * C_Y_delta_a
Y_delta_r = c17 * C_Y_delta_r

L_v = c8 * c9 + c11 * c10 + c12 * c6
L_p = gamma1 * q_trim + c12 * C_p_p # roll damping
L_r = -gamma2 * q_trim + c12 * C_p_r # di-hedral effect?

L_delta_a = c17 * C_p_delta_a
L_delta_r = c17 * C_p_delta_r

N_v = c7 * c14 + c10 * c15 + c16 * c6
N_p = gamma7 * q_trim + c12 * C_r_p # adverse yaw?
N_r = -gamma1 * q_trim + c12 * C_r_r # yaw damping

N_delta_a = c17 * C_r_delta_a
N_delta_r = c17 * C_r_delta_r

A_lat = np.array([
    [Y_v, Y_p, Y_r, g],
    [L_v, L_p, L_r, 0.0],
    [N_v, N_p, N_r, 0.0],
    [0.0, 1.0, 0.0, 0.0]
])

B_lat = np.array([
    [Y_delta_a, Y_delta_r],
    [L_delta_a, L_delta_r],
    [N_delta_a, N_delta_r],
    [0.0, 0.0]
])

# TODO: Get x and z aerodynamic derivatives (input is alpha trim).
(C_X_of_alpha, C_X_q_of_alpha, C_X_delta_e_of_alpha, \
    C_Z_of_alpha, C_Z_q_of_alpha, C_Z_delta_e_of_alpha) \
        = aero_coeffs.get_x_and_z_coefficients(alpha_trim)




if __name__ == "__main__":

    print("C_p_0 = ", C_p_0)
    print("C_p_beta = ", C_p_beta)
    print("C_p_p = ", C_p_p)
    print("C_p_r = ", C_p_r)
    print("C_p_delta_a = ", C_p_delta_a)
    print("C_p_delta_r = ", C_p_delta_r)

    print("C_r_0 = ", C_r_0)
    print("C_r_beta = ", C_r_beta)
    print("C_r_p = ", C_r_p)
    print("C_r_r = ", C_r_r)
    print("C_r_delta_a = ", C_r_delta_a)
    print("C_r_delta_r = ", C_r_delta_r)

    print("C_X_of_alpha = ", C_X_of_alpha)
    print("C_X_q_of_alpha = ", C_X_q_of_alpha)
    print("C_X_delta_e_of_alpha = ", C_X_delta_e_of_alpha)

    print("C_Z_of_alpha = ", C_Z_of_alpha)
    print("C_Z_q_of_alpha = ", C_Z_q_of_alpha)
    print("C_Z_delta_e_of_alpha = ", C_Z_delta_e_of_alpha)

    print("B_lat = ", B_lat)
    print("A_lat = ", A_lat)
