
import numpy as np
import models.aerosonde_uav as mav

class AerodynamicCoefficients:
    def __init__(self, aero_coeffs_list):
        if len(aero_coeffs_list) != 30:
            print("aero_coeffs_list should be length 30.")
        #----------------- Longitudinal ----------------#
        self._C_L_0 = aero_coeffs_list[0]
        self._C_D_0 =  aero_coeffs_list[1]
        self._C_m_0 =  aero_coeffs_list[2]
        self._C_L_alpha =  aero_coeffs_list[3]
        self._C_D_alpha =  aero_coeffs_list[4]
        self._C_m_alpha =  aero_coeffs_list[5]
        self._C_L_q =  aero_coeffs_list[6]
        self._C_D_q =  aero_coeffs_list[7]
        self._C_m_q =  aero_coeffs_list[8]
        self._C_L_delta_e =  aero_coeffs_list[9]
        self._C_D_delta_e =  aero_coeffs_list[10]
        self._C_m_delta_e =  aero_coeffs_list[11]
        # self._C_D_p = 0.0437
        #----------------- Lateral -----------------#
        self._C_Y_0 = aero_coeffs_list[12]
        self._C_l_0 = aero_coeffs_list[13]
        self._C_n_0 = aero_coeffs_list[14]
        self._C_Y_beta = aero_coeffs_list[15]
        self._C_l_beta = aero_coeffs_list[16]
        self._C_n_beta = aero_coeffs_list[17]
        self._C_Y_p = aero_coeffs_list[18]
        self._C_l_p = aero_coeffs_list[19]
        self._C_n_p = aero_coeffs_list[20]
        self._C_Y_r = aero_coeffs_list[21]
        self._C_l_r = aero_coeffs_list[22]
        self._C_n_r = aero_coeffs_list[23]
        self._C_Y_delta_a = aero_coeffs_list[24]
        self._C_l_delta_a = aero_coeffs_list[25]
        self._C_n_delta_a = aero_coeffs_list[26]
        self._C_Y_delta_r = aero_coeffs_list[27]
        self._C_l_delta_r = aero_coeffs_list[28]
        self._C_n_delta_r = aero_coeffs_list[29]

    def get_x_and_z_coefficients(self, alpha):
        
        C_L_0 = self._C_L_0
        C_L_alpha = self._C_L_alpha
        C_L_q = self._C_L_q
        C_L_delta_e = self._C_L_delta_e
        C_D_0 = self._C_D_0
        C_D_alpha = self._C_D_alpha
        C_D_q = self._C_D_q
        C_D_delta_e = self._C_D_delta_e

        # TODO: Try out nonlinear version of this.
        C_L_of_alpha = C_L_0 + C_L_alpha * alpha
        C_D_of_alpha = C_D_0 + C_D_alpha * alpha

        # Could rotate this after producing forces,
        # but might need these again for state space models.
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
    
    # TODO: Do unit tests on this (e.g., compare l and n with pdot and rdot).
    def get_p_and_r_derivatives(self,gamma3, gamma4, gamma8):

        C_l_0 = self._C_l_0
        C_l_beta = self._C_l_beta
        C_l_p = self._C_l_p
        C_l_r = self._C_l_r
        C_l_delta_a = self._C_l_delta_a
        C_l_delta_r = self._C_l_delta_r
        C_n_0 = self._C_n_0
        C_n_beta = self._C_n_beta
        C_n_p = self._C_n_p
        C_n_r = self._C_n_r
        C_n_delta_a = self._C_n_delta_a
        C_n_delta_r = self._C_n_delta_r

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


if __name__ == "__main__":

    aero_coeffs = AerodynamicCoefficients([
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

    (C_X_of_alpha, C_X_q_of_alpha, C_X_delta_e_of_alpha, \
        C_Z_of_alpha, C_Z_q_of_alpha, C_Z_delta_e_of_alpha) \
             = aero_coeffs.get_x_and_z_coefficients(np.deg2rad(5.0))

    print("C_X_of_alpha = ", C_X_of_alpha)
    print("C_X_q_of_alpha = ", C_X_q_of_alpha)
    print("C_X_delta_e_of_alpha = ", C_X_delta_e_of_alpha)
    print("C_Z_of_alpha = ", C_Z_of_alpha)
    print("C_Z_q_of_alpha = ", C_Z_q_of_alpha)
    print("C_Z_delta_e_of_alpha = ", C_Z_delta_e_of_alpha)

    gamma3 = 10.0
    gamma4 = 20.0
    gamma8 = 30.0

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

