
import numpy as np
from pytest import approx
import models.aerosonde_uav as ac

import models.mass_props as mp
from models.aerodynamics import AerodynamicCoefficients

def get_fx_fx_from_lift_drag(alpha, aero_coeffs, rho, Va, S, q, delta_e, c):

    C_L_of_alpha = aero_coeffs._C_L_0 + aero_coeffs._C_L_alpha * alpha
    C_D_of_alpha = aero_coeffs._C_D_0 + aero_coeffs._C_D_alpha * alpha

    C_D_q = aero_coeffs._C_D_q
    C_D_delta_e = aero_coeffs._C_D_delta_e
    C_L_q = aero_coeffs._C_L_q
    C_L_delta_e = aero_coeffs._C_L_delta_e

    q_bar = 0.5 * rho * Va**2

    F_drag = q_bar * S * (C_D_of_alpha + C_D_q * c / (2 * Va) * q + C_D_delta_e * delta_e)
    F_lift = q_bar * S * (C_L_of_alpha + C_L_q * c / (2 * Va) * q + C_L_delta_e * delta_e)

    DCM_wind2bod = np.array([
        [np.cos(alpha), 0.0, -np.sin(alpha)],
        [0.0, 1.0, 0.0],
        [np.sin(alpha), 0.0, np.cos(alpha)]
    ])

    Fxz_aero_wind = np.array([-F_drag, 0.0, -F_lift])

    Fxz_aero_bod = DCM_wind2bod @ Fxz_aero_wind

    return Fxz_aero_bod[0], Fxz_aero_bod[2]

def test_get_x_and_z_coefficients():

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
        ac.C_Y_delta_r, ac.C_l_delta_r, ac.C_n_delta_r \
    ])

    Va = 30.0 # [m/s]
    alpha = np.deg2rad(5.0)
    q = 10.0 # [rad/s]
    delta_e = 0.1
    c = 0.18994 # [m]
    S = 0.55 # [m^2]
    rho = 1.2682 # [kg/m^3]

    Fx_aero_bod_explicit, Fz_aero_bod_explicit = get_fx_fx_from_lift_drag(alpha, aero_coeffs, rho, Va, S, q, delta_e, c)

    print("Fx_aero_bod_explicit = ", Fx_aero_bod_explicit)
    print("Fz_aero_bod_explicit = ", Fz_aero_bod_explicit)

    (C_X_of_alpha, C_X_q_of_alpha, C_X_delta_e_of_alpha, \
        C_Z_of_alpha, C_Z_q_of_alpha, C_Z_delta_e_of_alpha) = aero_coeffs.get_x_and_z_coefficients(alpha)

    q_bar = 0.5 * rho * Va**2

    Fx_aero_bod = q_bar * S * (C_X_of_alpha + C_X_q_of_alpha * c / (2 * Va) * q + C_X_delta_e_of_alpha * delta_e)
    Fz_aero_bod = q_bar * S * (C_Z_of_alpha + C_Z_q_of_alpha * c / (2 * Va) * q + C_Z_delta_e_of_alpha * delta_e)

    print("Fx_aero_bod = ", Fx_aero_bod)
    print("Fz_aero_bod = ", Fz_aero_bod)

    assert Fx_aero_bod == approx(Fx_aero_bod_explicit, abs=1.0e-6)
    assert Fz_aero_bod == approx(Fz_aero_bod_explicit, abs=1.0e-6)
