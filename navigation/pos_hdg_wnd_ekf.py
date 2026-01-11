

import numpy as np

GRAV_ACCEL_MPS2 = 9.8067 # [m/s^2]

# TODO: Make generic EKF class where dynamics 
# and sensor models are passed in as arguments.

class PosHdgWndEKF:
    # x = [pn, pe, vg, chi, wn, we, psi]^T
    # u = [Va, q, r, phi, theta]^T
    def __init__(self, Q, R, T_out, N=1, x0=np.zeros(7), u0=np.zeros(5)):
        self._Q = Q
        self._R = R
        self._T_out = T_out
        self._N = N
        self._dt_step = T_out / N
        self._x_hat = x0
        self._P_hat = np.eye(len(x0))
        self._x_hat_pred = x0.copy() # is copy necessary?
        self._P_pred = np.eye(len(x0))
        self._x_hat_upd = x0.copy()
        self._P_upd = np.eye(len(x0))
        # TODO: Store fields you want to log.


    ####################################################
    #   Dynamics (really kinematics) and Sensor Models
    ####################################################

    # f(x,u)
    # x = [pn, pe, vg, chi, wn, we, psi]^T
    # u = [Va, q, r, phi, theta]^T
    def get_x_dot(self, x, u):
        pn, pe, vg, chi, wn, we, psi = x
        va, q, r, phi, theta = u
        g = GRAV_ACCEL_MPS2

        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        cc = np.cos(chi)
        sc = np.sin(chi)
        cs = np.cos(psi)
        ss = np.sin(psi)

        psi_dot = q*sp/ct + r*cp/ct

        pn_dot = vg * cc
        pe_dot = vg * sc
        vg_dot = ((va * cs + wn)*(-va * psi_dot * ss) + (va * ss + we)*(va * psi_dot * cs)) / vg
        chi_dot = (g / vg) * np.tan(phi) * np.cos(chi - psi)
        wn_dot = 0
        we_dot = 0
        
        return np.array([pn_dot, pe_dot, vg_dot, chi_dot, wn_dot, we_dot, psi_dot])


    ####################################################
                #   Jacobians for EKF
    ####################################################

    def get_J_df_dx(self, x, u):
        pn, pe, vg, chi, wn, we, psi = x
        va, q, r, phi, theta = u
        g = GRAV_ACCEL_MPS2

        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)
        cc = np.cos(chi)
        sc = np.sin(chi)
        cs = np.cos(psi) # cuz, ya know, "sigh"
        ss = np.sin(psi)

        _, _, vg_dot, chi_dot, _, _, psi_dot = self.get_x_dot(x, u)

        J_df_dx = np.zeros((len(x), len(x)))

        # dpn_dx
        J_df_dx[0,2] = cc
        J_df_dx[0,3] = -vg * sc

        # dpe_dx
        J_df_dx[1,2] = sc
        J_df_dx[1,3] = vg * cc

        # dvg_dx
        J_df_dx[2,2] = -vg_dot / vg
        # TODO: Check these (textbook doesn't have '/ vg' in the denominator).
        J_df_dx[2,4] = -psi_dot * va * ss / vg
        J_df_dx[2,5] = psi_dot * va * cs / vg
        J_df_dx[2,6] = -psi_dot * va * (wn * cs + we * ss) / vg

        # dchi_dx
        J_df_dx[3,2] = -chi_dot / vg
        J_df_dx[3,3] = -(g / vg) * np.tan(phi) * sc / np.cos(chi - psi)
        J_df_dx[3,6] = (g / vg) * np.tan(phi) * sc / np.cos(chi - psi)

        return J_df_dx

    def get_J_dh_dx(self, x, u):
        phi, theta = x
        Va, p, q, r = u
        g = GRAV_ACCEL_MPS2

        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)

        dax_sens_dphi = 0.0
        dax_sens_dtheta = (q * Va + g) * ct

        day_sens_dphi = -g * ct * cp
        day_sens_dtheta = -r * Va * st - p * Va * ct + g * st * sp

        daz_sens_dphi = g * ct * sp
        daz_sens_dtheta = (q * Va + g * cp) * st

        return np.array([
            [dax_sens_dphi, dax_sens_dtheta],
            [day_sens_dphi, day_sens_dtheta],
            [daz_sens_dphi, daz_sens_dtheta],
        ])


    ####################################################
                    #   EKF Algo
    ####################################################

    def predict(self,x_hat, P, u):
        Q = self._Q
        dt_step = self._dt_step
        for _ in range(self._N):
            x_hat = x_hat + dt_step * self.get_x_dot(x_hat, u)
            A = self.get_J_df_dx(x_hat, u)
            P = P + dt_step * (A @ P + P @ A.T + Q)
        return x_hat, P

    def update(self, x_hat_pred, P_pred, u, y_meas):
        R = self._R
        C = self.get_J_dh_dx(x_hat_pred, u)
        L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
        y_est = self.get_y_accel_est(x_hat_pred, u)
        y_err = y_meas - y_est
        x_err = L @ y_err
        x_hat_upd = x_hat_pred + x_err
        P_upd = (np.eye(len(x_hat_pred)) - L @ C) @ P_pred
        return x_hat_upd, P_upd

    def ekf_step(self, x_hat, P_hat, u, y_meas):
        x_hat_pred, P_pred = self.predict(x_hat, P_hat, u)
        x_hat_upd, P_upd = self.update(x_hat_pred, P_pred, u, y_meas)
        return x_hat_pred, P_pred, x_hat_upd, P_upd
