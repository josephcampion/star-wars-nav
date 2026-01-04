

import numpy as np

GRAV_ACCEL_MPS2 = 9.8067 # [m/s^2]

# TODO: Make generic EKF class where dynamics 
# and sensor models are passed in as arguments.

class PitchRollEKF:
    # x = [phi, theta]^T
    # u = [Va, p, q, r]^T
    def __init__(self, Q, R, T_out, N=1, x0=np.array([0.0, 0.0]), u0=np.array([0.0, 0.0, 0.0, 0.0])):
        self._Q = Q
        self._R = R
        self._T_out = T_out
        self._N = N
        self._dt_step = T_out / N
        self._x_hat = x0
        self._P_hat = np.eye(2)
        self._x_hat_pred = x0.copy() # is copy necessary?
        self._P_pred = np.eye(2)
        self._x_hat_upd = x0.copy()
        self._P_upd = np.eye(2)
        # TODO: Store fields you want to log.


    ####################################################
    #   Dynamics (really kinematics) and Sensor Models
    ####################################################

    # f(x,u)
    # x = [phi, theta]^T
    # u = [Va, p, q, r]^T
    def get_x_dot(self, x, u):
        phi, theta = x
        _, p, q, r = u

        sp = np.sin(phi)
        cp = np.cos(phi)
        tt = np.tan(theta)
        
        phi_dot = p + q * sp * tt + r * cp * tt
        theta_dot = q * cp - r * sp
        
        # TODO: Store x_dot in class?
        return np.array([phi_dot, theta_dot])

    # h(x,u)
    # x = [phi, theta]^T
    # u = [Va, p, q, r]^T
    def get_y_accel_est(self, x, u):
        phi, theta = x
        Va, p, q, r = u
        g = GRAV_ACCEL_MPS2

        st = np.sin(theta)
        ct = np.cos(theta)
        sp = np.sin(phi)
        cp = np.cos(phi)

        y_accel_x_est = q * Va * st + g * st
        y_accel_y_est = r * Va * ct - p * Va * st - g * ct * sp
        y_accel_z_est = -q * Va * ct - g * ct * cp

        # TODO: Store output y in class?
        return np.array([y_accel_x_est, y_accel_y_est, y_accel_z_est])


    ####################################################
                #   Jacobians for EKF
    ####################################################

    def get_J_df_dx(self, x, u):

        phi, theta = x
        _, p, q, r = u

        cp = np.cos(phi)
        sp = np.sin(phi)
        tt = np.tan(theta)
        ct = np.cos(theta)

        dphi_dot_dphi = q * cp * tt - r * sp * tt
        # TODO: Figure out why textbook has -r*cp instead.
        dphi_dot_dtheta = (q * sp + r * cp) / (ct**2)
        dtheta_dot_dphi = -q * sp - r * cp
        dtheta_dot_dtheta = 0.0

        return np.array([
            [dphi_dot_dphi, dphi_dot_dtheta],
            [dtheta_dot_dphi,dtheta_dot_dtheta]
        ])

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

####################################################
        #   Unused Helper Functions
####################################################

# def wrap_to_pi_minus_pi(x):
#     return np.arctan2(np.sin(x), np.cos(x))

# def condition_residual(x_err):
#     pitch_residual = wrap_to_pi_minus_pi(x_err[0])
#     roll_residual = np.clip(x_err[1], -np.deg2rad(20.0), np.deg2rad(20.0))
#     return np.array([pitch_residual, roll_residual])

# def get_innovation(y_meas, x_hat_pred, u):
#     y_est = get_y_accel_est(x_hat_pred, u)
#     y_err = y_meas - y_est
#     return y_err

# def get_residual(L, y_err):
#     x_err = L @ y_err
#     return x_err # condition_residual(x_err)

# def get_kalman_gain(x_hat_pred, P_pred, R, u):
#     C = get_J_dh_dx(x_hat_pred, u)
#     L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
#     return C, L

