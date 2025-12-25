
import numpy as np

class GammaInertiaTerms():
    def __init__(self, Jx, Jy, Jz, Jxz):

        # TODO: Replace with cross product/matrix calc
        # or do that with a unit test...
        gamma = Jx * Jz - Jxz**2
        gamma1 = Jxz * (Jx - Jy + Jz) / gamma
        gamma2 = (Jz * (Jz - Jy) + Jxz**2) / gamma
        gamma3 = Jz / gamma
        gamma4 = Jxz / gamma
        gamma5 = (Jz - Jx) / Jy
        gamma6 = Jxz / Jy
        gamma7 = ((Jx - Jy) * Jx + Jxz**2) / gamma
        gamma8 = Jx / gamma
        
        self._Gamma = np.array([gamma, gamma1, gamma2, gamma3, \
            gamma4, gamma5, gamma6, gamma7, gamma8])

    def get_gamma_vec(self):
        return self._Gamma

# TODO: Do unit tests on this, and compare with simple rotation math.
def get_X_and_Z_derivatives(alpha, \
    C_L_0, C_L_alpha, C_L_q, C_L_delta_e, \
    C_D_0, C_D_alpha, C_D_q, C_D_delta_e):

    C_L_of_alpha = C_L_0 + C_L_alpha * alpha
    C_D_of_alpha = C_D_0 + C_D_alpha * alpha 

    C_X_of_alpha = -C_D_of_alpha * np.cos(alpha) + C_L_of_alpha * np.sin(alpha)
    C_X_q_of_alpha = -C_D_q * np.cos(alpha) + C_L_q * np.sin(alpha)
    C_X_delta_e_of_alpha = -C_D_delta_e * np.cos(alpha) + \
        C_L_delta_e * np.sin(alpha)

    C_Z_of_alpha = -C_D_of_alpha * np.sin(alpha) - C_L_of_alpha * np.cos(alpha)
    C_Z_q_of_alpha = -C_D_q * np.sin(alpha) - C_L_q * np.cos(alpha)
    C_Z_delta_e_of_alpha = -C_D_delta_e * np.sin(alpha) - \
        C_L_delta_e * np.cos(alpha)

    return np.array([
        C_X_of_alpha,
        C_X_q_of_alpha,
        C_X_delta_e_of_alpha,
        C_Z_of_alpha,
        C_Z_q_of_alpha,
        C_Z_delta_e_of_alpha,
    ])

# TODO: Do unit tests on this, and compare with rotation math.
def get_p_and_r_derivatives(gamma3, gamma4, gamma8, \
    C_l_0, C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r, \
    C_n_0, C_n_beta, C_n_p, C_n_r, C_n_delta_a, C_n_delta_r):

    C_p_0 = gamma3 * C_l_0 + gamma4 * C_n_0
    C_p_beta = gamma3 * C_l_beta + gamma4 * C_n_beta
    C_p_p = gamma3 * C_l_p + gamma4 * C_n_p
    C_p_r = gamma3 * C_l_r + gamma4 * C_n_r
    C_p_delta_a = gamma3 * C_l_delta_a + gamma4 * C_n_delta_a
    C_p_delta_r = gamma3 * C_l_delta_r + gamma4 * C_n_delta_r

    C_r_0 = gamma4 * C_l_0 + gamma8 * C_n_0
    C_r_beta = gamma4 * C_l_beta + gamma8 * C_n_beta
    C_r_p = gamma4 * C_l_p + gamma8 * C_n_p
    C_r_r = gamma4 * C_l_r + gamma8 * C_n_r
    C_r_delta_a = gamma4 * C_l_delta_a + gamma8 * C_n_delta_a
    C_r_delta_r = gamma4 * C_l_delta_r + gamma8 * C_n_delta_r

    return np.array([
        C_p_0, C_p_beta, C_p_p, C_p_r, C_p_delta_a, C_p_delta_r, \
        C_r_0, C_r_beta, C_r_p, C_r_r, C_r_delta_a, C_r_delta_r
    ])
