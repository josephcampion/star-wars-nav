

import numpy as np
import matplotlib.pyplot as plt

import simulation.kinematics as kin
from models.sensors import Accelerometer, Gyroscope

####################################################
#   Kinematics (TODO: Re-use kin and model.sensors)
####################################################

# f(x,u)
# x = [phi, theta]^T
# u = [Va, p, q, r]^T
def get_x_dot(p, q, r, phi, theta):
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
    g = 9.81 # [m/s^2]

    y_accel_x_est = q * Va * np.sin(theta) + g * np.sin(theta)
    y_accel_y_est = r * Va * np.cos(theta) - p * Va * np.sin(theta) - g * np.cos(theta) * np.sin(phi)
    y_accel_z_est = -q * Va * np.cos(theta) - g * np.cos(theta) * np.cos(phi)

    return np.array([y_accel_x_est, y_accel_y_est, y_accel_z_est])

# x = [u, v, w, phi, theta, p, q, r]
# xdot = [udot, vdot, wdot]
def get_y_accel_meas(x, xdot): # noise is coming from sensor models

    u, v, w = x[0:3]
    phi, theta = x[3:5]
    p, q, r = x[5:8]

    udot, vdot, wdot = xdot

    g = 9.81

    y_accel_x  = udot + q*w - r*v + g*np.sin(theta)
    y_accel_y = vdot + r*u - p*w - g*np.cos(theta)*np.sin(phi)
    y_accel_z = wdot + p*v - q*u - g*np.cos(theta)*np.cos(phi)

    return np.array([y_accel_x, y_accel_y, y_accel_z])


####################################################
            #   Jacobians for EKF
####################################################

def get_J_df_dx(x, u):

    phi, theta = x
    p, q, r = u

    cp = np.cos(phi)
    sp = np.sin(phi)
    tt = np.tan(theta)
    ct = np.cos(theta)

    dphi_dot_dphi = q*cp*tt - r*sp*tt
    dphi_dot_dtheta = (q*sp + r*cp) / (ct**2)
    dtheta_dot_dphi = -q*sp - r*cp
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
    dax_sens_dtheta = -g*ct*cp
    day_sens_dphi = g*ct*sp
    day_sens_dtheta = q*Va*ct + g*cp
    daz_sens_dphi = -r*Va*st - p*Va*ct + g*st*sp
    daz_sens_dtheta = q*Va*sp + g*st*cp

    return np.array([
        [dax_sens_dphi, dax_sens_dtheta],
        [day_sens_dphi, day_sens_dtheta],
        [daz_sens_dphi, daz_sens_dtheta],
    ])


####################################################
                #   EKF Algo
####################################################

def ekf_step(x_hat, P_hat, Q, R, u, y_meas, T_out, N=1):

    # Predict step
    dt_step = T_out / N
    x_hat_pred = x_hat + dt_step * get_x_dot(x_hat, u)
    A = get_J_df_dx(x_hat, u)
    P_pred = P_hat + dt_step * (A @ P_hat + P_hat @ A.T + Q)

    # Update step
    C = get_J_dh_dx(x_hat_pred, u)
    L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
    y_est = get_y_accel_est(x_hat_pred, u)
    x_hat_upd = x_hat_pred + L @ (y_meas - y_est)
    P_upd = (np.eye(len(x_hat_pred)) - L @ C) @ P_pred

    return x_hat_upd, P_upd


####################################################
    #   Initialize Simulation Parameters
####################################################

Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Set truth motion for EKF testing
u0 = 30.0 # [m/s]
w0 = 1.0 # [m/s]
u_truth = u0 * np.ones(nt) + 3.0 / Tsim * np.linspace(t0,Tsim,nt)
v_truth = np.zeros(nt)
w_truth = w0 * np.ones(nt) - 1.0 / Tsim * np.linspace(t0,Tsim,nt)

p_truth = np.deg2rad(1.0) * np.sin(3*time)
q_truth = -np.deg2rad(2.0) * np.sin(2*time)
r_truth = np.deg2rad(3.0) * np.sin(1*time)
# r_truth = 3.0 * np.sin(1*time)

phi0 = np.deg2rad(-3.0)
theta0 = np.deg2rad(5.0)

phi_truth = phi0 + np.zeros(nt)
theta_truth = theta0 + np.zeros(nt)
# psi_truth = np.zeros(nt)

phi_dot_truth = np.zeros(nt)
theta_dot_truth = np.zeros(nt)

####################################################
    #   Initialize Sensor Models
####################################################

# Make accelerometer sensor models
accel_bias = 1.e-2
accel_noise = 1.e-2
x_accel = Accelerometer(accel_bias, accel_noise)
y_accel = Accelerometer(accel_bias, accel_noise)
z_accel = Accelerometer(accel_bias, accel_noise)

# Make gyroscope sensor models
gyro_bias = 1.e-3
gyro_noise = 1.e-3
x_gyro = Gyroscope(gyro_bias, gyro_noise)
y_gyro = Gyroscope(gyro_bias, gyro_noise)
z_gyro = Gyroscope(gyro_bias, gyro_noise)

y_accel_x_meas = np.zeros(nt)
y_accel_y_meas = np.zeros(nt)
y_accel_z_meas = np.zeros(nt)

x_gyro_meas = np.zeros(nt)
y_gyro_meas = np.zeros(nt)
z_gyro_meas = np.zeros(nt)

####################################################
    #   Initialize EKF Estimation
####################################################

x_hat = np.array([phi0, theta0])
P_hat = np.eye(2)
Q = np.eye(2) * 1.e-6
R = np.eye(3) * 1.e-2

x_hat_log = np.zeros((nt, 2))
P_log = np.zeros((nt, 2, 2))

####################################################
    #   Run Simulation of Sensor Models
####################################################

for i in range(nt):

    phi, theta = phi_truth[i], theta_truth[i]
    u, v, w = u_truth[i], v_truth[i], w_truth[i]
    p, q, r = p_truth[i], q_truth[i], r_truth[i]

    phi_dot, theta_dot = get_x_dot(p, q, r, phi, theta)

    phi_dot_truth[i] = phi_dot
    theta_dot_truth[i] = theta_dot

    # Get sensor measurements
    x_gyro_meas[i] = x_gyro.get_sensor_data(p_truth[i])
    y_gyro_meas[i] = y_gyro.get_sensor_data(q_truth[i])
    z_gyro_meas[i] = z_gyro.get_sensor_data(r_truth[i])

    # Insert truth motion into sensor models
    # x = [u, v, w, phi, theta, p, q, r]
    # xdot = [udot, vdot, wdot]
    x_accel_in = np.array([u, v, w, phi, theta, p, q, r])
    # TODO: Insert x_dot_accel_in from kinematics.
    x_dot_accel_in = np.zeros(3) # np.array([r])
    y_accel_meas = get_y_accel_meas(x_accel_in, x_dot_accel_in)

    y_accel_x_meas[i] = x_accel.get_sensor_data(y_accel_meas[0])
    y_accel_y_meas[i] = y_accel.get_sensor_data(y_accel_meas[1])
    y_accel_z_meas[i] = z_accel.get_sensor_data(y_accel_meas[2])

    # TODO: Use airspeed sensor model to get Va (and insert wind!).
    Va = np.sqrt(u**2 + v**2 + w**2)

    """
    # Run EKF step
    y_accel_meas = np.array([y_accel_x_meas[i], y_accel_y_meas[i], y_accel_z_meas[i]])
    x_hat, P_hat = ekf_step(x_hat, P_hat, Q, R, np.array([Va, p, q, r]), y_accel_meas, dt)

    x_hat_log[i] = x_hat
    P_log[i] = P_hat
    """

    if i < (nt-1): # propagate truth motion
        phi_truth[i+1] = phi_truth[i] + phi_dot * dt
        theta_truth[i+1] = theta_truth[i] + theta_dot * dt


####################################################
            #   Plot Truth Rates
####################################################

_, axs = plt.subplots(3,2)

axs[0,0].plot(time, u_truth, label='Truth')
axs[0,0].grid(True)
# axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('[rad/s]')
axs[0,0].set_title(r'$u$')
axs[0,0].legend()

axs[1,0].plot(time, v_truth, label='Truth')
axs[1,0].grid(True)
# axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('[rad/s]')
axs[1,0].set_title(r'$v$')
axs[1,0].legend()

axs[2,0].plot(time, w_truth, label='Truth')
axs[2,0].grid(True)
axs[2,0].set_xlabel('Time [s]')
axs[2,0].set_ylabel('[rad/s]')
axs[2,0].set_title(r'$w$')
axs[2,0].legend()

axs[0,1].plot(time, p_truth, label='Truth')
axs[0,1].plot(time, x_gyro_meas, label='Sensor', linestyle=':')
axs[0,1].grid(True)
# axs[0,1].set_xlabel('Time [s]')
axs[0,1].set_ylabel('[rad/s]')
axs[0,1].set_title(r'$p$ vs. $p_{meas}$')
axs[0,1].legend()

axs[1,1].plot(time, q_truth, label='Truth')
axs[1,1].plot(time, y_gyro_meas, label='Sensor', linestyle=':')
axs[1,1].grid(True)
# axs[1,1].set_xlabel('Time [s]')
axs[1,1].set_ylabel('[rad/s]')
axs[1,1].set_title(r'$q$ vs. $q_{meas}$')
axs[1,1].legend()

axs[2,1].plot(time, r_truth, label='Truth')
axs[2,1].plot(time, z_gyro_meas, label='Sensor', linestyle=':')
axs[2,1].grid(True)
axs[2,1].set_xlabel('Time [s]')
axs[2,1].set_ylabel('[rad/s]')
axs[2,1].set_title(r'$r$ vs. $r_{meas}$')
axs[2,1].legend()

plt.suptitle('Attitude EKF Test')


####################################################
    #   Plot Attitude (TODO: Estimation)
####################################################

_, axs = plt.subplots(2,2)

axs[0,0].plot(time, phi_truth, label='Truth')
axs[0,0].grid(True)
axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('[rad/s]')
axs[0,0].set_title(r'$\phi$')
axs[0,0].legend()

axs[1,0].plot(time, phi_dot_truth, label='Truth')
axs[1,0].grid(True)
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('[rad/s]')
axs[1,0].set_title(r'$\dot{\phi}$')
axs[1,0].legend()

axs[0,1].plot(time, theta_truth, label='Truth')
axs[0,1].grid(True)
axs[0,1].set_xlabel('Time [s]')
axs[0,1].set_ylabel('[rad/s]')
axs[0,1].set_title(r'$\theta$')
axs[0,1].legend()

axs[1,1].plot(time, theta_dot_truth, label='Truth')
axs[1,1].grid(True)
axs[1,1].set_xlabel('Time [s]')
axs[1,1].set_ylabel('[rad/s]')
axs[1,1].set_title(r'$\dot{\theta}$')
axs[1,1].legend()

plt.suptitle('Attitude EKF Test')

plt.show()
