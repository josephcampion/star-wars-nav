
import numpy as np
import models.aerosonde_uav as ac
from models.aerodynamics import AerodynamicCoefficients
from models.propulsion import EngineProperties
from models.mass_props import MassProperties
import flight_controls.trim as trim

AIR_DENSITY_KGM3 = 1.2682 # [kg/m^3] # TODO: Make this a function of altitude?

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

    # TODO: Get gamma3/4/8 from mass props to convert 
    # aerodynamic coefficients into p and r derivatives.
    gamma_vec = mass_props.get_gamma_vec()
    gamma3 = gamma_vec[3]
    gamma4 = gamma_vec[4]
    gamma8 = gamma_vec[8]

    (C_p_0, C_p_beta, C_p_p, C_p_r, C_p_delta_a, C_p_delta_r, \
        C_r_0, C_r_beta, C_r_p, C_r_r, C_r_delta_a, C_r_delta_r) \
            = aero_coeffs.get_p_and_r_derivatives(gamma3, gamma4, gamma8)
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

    # TODO: Get x and z aerodynamic derivatives (input is alpha trim).
    (C_X_of_alpha, C_X_q_of_alpha, C_X_delta_e_of_alpha, \
        C_Z_of_alpha, C_Z_q_of_alpha, C_Z_delta_e_of_alpha) \
            = aero_coeffs.get_x_and_z_coefficients(alpha_trim)
    print("C_X_of_alpha = ", C_X_of_alpha)
    print("C_X_q_of_alpha = ", C_X_q_of_alpha)
    print("C_X_delta_e_of_alpha = ", C_X_delta_e_of_alpha)
    print("C_Z_of_alpha = ", C_Z_of_alpha)
    print("C_Z_q_of_alpha = ", C_Z_q_of_alpha)
    print("C_Z_delta_e_of_alpha = ", C_Z_delta_e_of_alpha)

    # TODO: Make state space terms for lateral dynamics.
    g = 9.81 # [m/s^2]
    Y_v = rho * S_prop * v_trim / (4 * m * Va_trim) * (C_Y_p * p_trim + C_Y_r * r_trim) + \
        rho * S_prop * v_trim / m * (C_Y_0 + C_Y_beta * beta_trim + C_Y_delta_a * delta_a_trim + C_Y_delta_r * delta_r_trim) + \
        rho * S_prop * C_Y_beta / (2 * m) * Va_trim
    Y_p = 0.0
    Y_r = 1.0
    L_v = -0.1
    L_p = -4.0 # roll damping
    L_r = 0.7 # di-hedral effect?
    N_v = 0.05
    N_p = -0.5 # adverse yaw?
    N_r = -0.3 # yaw damping
    A_lat = np.array([
        [Y_v, Y_p, Y_r, g],
        [L_v, L_p, L_r, 0.0],
        [N_v, N_p, N_r, 0.0],
        [0.0, 1.0, 0.0, 0.0]
    ])
    Y_da = 0.0
    Y_dr = 0.2
    L_da = 10.0
    L_dr = 1.0
    N_da = 0.5
    N_dr = -5.0
    B_lat = np.array([
        [Y_da, Y_dr],
        [L_da, L_dr],
        [N_da, N_dr],
        [0.0, 0.0]
    ])

    # print(dir(mass_props))
    # print(dir(engine_props))
    # print(dir(aero_coeffs))