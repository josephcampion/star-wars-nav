

import numpy as np

GRAV_ACCEL_MPS2 = 9.8067 # [m/s^2]

####################################################
#   Kinematics (TODO: Re-use kin and model.sensors)
####################################################

# f(x,u)
# x = [phi, theta]^T
# u = [Va, p, q, r]^T
def get_x_dot(x, u):
    phi, theta = x
    _, p, q, r = u

    sp = np.sin(phi)
    cp = np.cos(phi)
    tt = np.tan(theta)
    
    phi_dot = p + q * sp * tt + r * cp * tt
    theta_dot = q * cp - r * sp
    
    return np.array([phi_dot, theta_dot])

# TODO: Make a unit test for this (vs. kinematics for no winds case).
# h(x,u)
# x = [phi, theta]^T
# u = [Va, p, q, r]^T
def get_y_accel_est(x, u):
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

    return np.array([y_accel_x_est, y_accel_y_est, y_accel_z_est])

# x = [u, v, w, phi, theta, p, q, r]
# xdot = [udot, vdot, wdot]
def get_y_accel_meas(x, xdot=np.zeros(3)): # noise is coming from sensor models

    u, v, w = x[0:3]
    phi, theta = x[3:5]
    p, q, r = x[5:8]

    udot, vdot, wdot = xdot

    g = GRAV_ACCEL_MPS2

    y_accel_x  = udot + q*w - r*v + g * np.sin(theta)
    y_accel_y = vdot + r*u - p*w - g * np.cos(theta) * np.sin(phi)
    y_accel_z = wdot + p*v - q*u - g * np.cos(theta) * np.cos(phi)

    return np.array([y_accel_x, y_accel_y, y_accel_z])


####################################################
            #   Jacobians for EKF
####################################################

def get_J_df_dx(x, u):

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

def get_J_dh_dx(x, u):
    phi, theta = x
    Va, p, q, r = u
    g = 9.81 # [m/s^2]

    cp = np.cos(phi)
    sp = np.sin(phi)
    ct = np.cos(theta)
    st = np.sin(theta)

    dax_sens_dphi = 0.0
    dax_sens_dtheta = -g * ct * cp
    day_sens_dphi = g * ct * sp
    day_sens_dtheta = q * Va * ct + g * ct
    daz_sens_dphi = -r * Va * st - p * Va * ct + g * st * sp
    daz_sens_dtheta = (q * Va + g * cp) * st

    return np.array([
        [dax_sens_dphi, dax_sens_dtheta],
        [day_sens_dphi, day_sens_dtheta],
        [daz_sens_dphi, daz_sens_dtheta],
    ])


####################################################
                #   EKF Algo
####################################################

def wrap_to_pi_minus_pi(x):
    return np.arctan2(np.sin(x), np.cos(x))

def condition_residual(x_err):
    pitch_residual = wrap_to_pi_minus_pi(x_err[0])
    roll_residual = np.clip(x_err[1], -np.deg2rad(20.0), np.deg2rad(20.0))
    return np.array([pitch_residual, roll_residual])

def get_innovation(y_meas, x_hat_pred, u):
    y_est = get_y_accel_est(x_hat_pred, u)
    y_err = y_meas - y_est
    return y_err

def get_residual(L, y_err):
    x_err = L @ y_err
    return x_err # condition_residual(x_err)

def ekf_predict(x_hat, P_hat, Q, u, T_out, N=1):
    dt_step = T_out / N
    for i in range(N):
        x_hat_pred = x_hat + dt_step * get_x_dot(x_hat, u)
        A = get_J_df_dx(x_hat, u)
        P_pred = P_hat + dt_step * (A @ P_hat + P_hat @ A.T + Q)
    return x_hat_pred, P_pred

def get_kalman_gain(x_hat_pred, P_pred, R, u):
    C = get_J_dh_dx(x_hat_pred, u)
    L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
    return C, L

def ekf_update(x_hat_pred, P_pred, R, u, y_meas):
    C, L = get_kalman_gain(x_hat_pred, P_pred, R, u)
    y = get_innovation(y_meas, x_hat_pred, u)
    x_hat_upd = x_hat_pred + get_residual(L, y)
    P_upd = (np.eye(len(x_hat_pred)) - L @ C) @ P_pred

    return x_hat_upd, P_upd

def ekf_step(x_hat, P_hat, Q, R, u, y_meas, T_out, N=1):

    x_hat_pred, P_pred = ekf_predict(x_hat, P_hat, Q, u, T_out, N)
    
    x_hat_upd, P_upd = ekf_update(x_hat_pred, P_pred, R, u, y_meas)

    return x_hat_upd, P_upd

