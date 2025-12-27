
import numpy as np
import models.aerosonde_uav as mav

# TODO: Add steady winds and gusts.

# def get_airspeed_vector(u, v, w, uw, vw, ww):
#     return np.array([u - uw, v - vw, w - ww])

def get_Va(u, v, w):
    return np.sqrt(u**2 + v**2 + w**2)

def get_alpha(u, v, w):
    return np.arctan2(w, u)

def get_beta(u, v, w):
    Va = get_Va(u, v, w)
    return np.arcsin(v / Va)


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

    def get_lift_force(self, alpha, q, delta_e, rho, Va, S, c):
        C_L_0 = self._C_L_0
        C_L_alpha = self._C_L_alpha
        C_L_q = self._C_L_q
        C_L_delta_e = self._C_L_delta_e

        # TODO: Try out nonlinear version of this.
        C_L_of_alpha = C_L_0 + C_L_alpha * alpha

        q_bar = 0.5 * rho * Va**2

        F_lift = q_bar * S * (C_L_of_alpha + C_L_q * c / (2 * Va) * q + C_L_delta_e * delta_e)

        return F_lift

    def get_drag_force(self, alpha, q, delta_e, rho, Va, S, c):
        C_D_0 = self._C_D_0
        C_D_alpha = self._C_D_alpha
        C_D_q = self._C_D_q
        C_D_delta_e = self._C_D_delta_e

        C_D_of_alpha = C_D_0 + C_D_alpha * alpha

        q_bar = 0.5 * rho * Va**2

        F_drag = q_bar * S * (C_D_of_alpha + C_D_q * c / (2 * Va) * q + C_D_delta_e * delta_e)

        return F_drag

    def get_side_force(self, beta, p, r, delta_a, delta_r, rho, Va, S, b):
        C_Y_0 = self._C_Y_0
        C_Y_beta = self._C_Y_beta
        C_Y_p = self._C_Y_p
        C_Y_r = self._C_Y_r
        C_Y_delta_a = self._C_Y_delta_a
        C_Y_delta_r = self._C_Y_delta_r

        q_bar = 0.5 * rho * Va**2

        k_omega = b / (2 * Va)

        F_side = q_bar * S * (C_Y_0 + C_Y_beta * beta + C_Y_p * k_omega * p + C_Y_r * k_omega * r + C_Y_delta_a * delta_a + C_Y_delta_r * delta_r)

        return F_side

    def get_fx_and_fz(self, alpha, q, delta_e, rho, Va, S, c):
        F_lift = self.get_lift_force(alpha, q, delta_e, rho, Va, S, c)
        F_drag = self.get_drag_force(alpha, q, delta_e, rho, Va, S, c)

        F_x = -F_drag * np.cos(alpha) + F_lift * np.sin(alpha)
        F_z = -F_drag * np.sin(alpha) - F_lift * np.cos(alpha)

        return F_x, F_z

    def get_roll_moment(self, beta, p, r, delta_a, delta_r, rho, Va, S, b):
        C_l_0 = self._C_l_0
        C_l_beta = self._C_l_beta
        C_l_p = self._C_l_p
        C_l_r = self._C_l_r
        C_l_delta_a = self._C_l_delta_a
        C_l_delta_r = self._C_l_delta_r
        
        q_bar = 0.5 * rho * Va**2

        k_w = b / (2 * Va)

        M_roll = q_bar * S * b * (C_l_0 + C_l_beta * beta + C_l_p * k_w * p + C_l_r * k_w * r + C_l_delta_a * delta_a + C_l_delta_r * delta_r)

        return M_roll
    
    def get_yaw_moment(self, beta, p, r, delta_a, delta_r, rho, Va, S, b):
        C_n_0 = self._C_n_0
        C_n_beta = self._C_n_beta
        C_n_p = self._C_n_p
        C_n_r = self._C_n_r
        C_n_delta_a = self._C_n_delta_a
        C_n_delta_r = self._C_n_delta_r
        
        q_bar = 0.5 * rho * Va**2

        k_w = b / (2 * Va)

        M_yaw = q_bar * S * b * (C_n_0 + C_n_beta * beta + C_n_p * k_w * p + C_n_r * k_w * r + C_n_delta_a * delta_a + C_n_delta_r * delta_r)

        return M_yaw
    
    def get_pitch_moment(self, alpha, q, delta_e, rho, Va, S, c):
        C_m_0 = self._C_m_0
        C_m_alpha = self._C_m_alpha
        C_m_q = self._C_m_q
        C_m_delta_e = self._C_m_delta_e
        
        q_bar = 0.5 * rho * Va**2

        M_pitch = q_bar * S * c * (C_m_0 + C_m_alpha * alpha + C_m_q * c / (2 * Va) * q + C_m_delta_e * delta_e)

        return M_pitch

    def get_aero_forces_and_moments(self, alpha, beta, Va, pqr, control, rho, S, c, b):

        p, q, r = pqr

        delta_e, _, delta_a, delta_r = control[0:4]

        F_lift = self.get_lift_force(alpha, q, delta_e, rho, Va, S, c)
        F_drag = self.get_drag_force(alpha, q, delta_e, rho, Va, S, c)

        F_x = -F_drag * np.cos(alpha) + F_lift * np.sin(alpha)
        F_z = -F_drag * np.sin(alpha) - F_lift * np.cos(alpha)

        F_y = self.get_side_force(beta, p, r, delta_a, delta_r, rho, Va, S, b)

        M_roll = self.get_roll_moment(beta, p, r, delta_a, delta_r, rho, Va, S, b)
        M_yaw = self.get_yaw_moment(beta, p, r, delta_a, delta_r, rho, Va, S, b)
        M_pitch = self.get_pitch_moment(alpha, q, delta_e, rho, Va, S, c)

        F_aero = np.array([F_x, F_y, F_z])
        M_aero = np.array([M_roll, M_yaw, M_pitch])

        return F_aero, M_aero


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

