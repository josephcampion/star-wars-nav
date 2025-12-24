
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