
import numpy as np
import matplotlib.pyplot as plt

import models.sensors as sens
import navigation.pitch_roll_ekf as ekf

Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Set truth motion for EKF testing
u0 = 25.0 # [m/s]
w0 = 2.0 # [m/s]
u_truth = u0 * np.ones(nt) + 0.0 / Tsim * np.linspace(t0,Tsim,nt)
v_truth = np.zeros(nt)
w_truth = w0 * np.ones(nt) - 0.0 / Tsim * np.linspace(t0,Tsim,nt)

p_truth = np.deg2rad(5.0) * np.sin(0.25*time)
q_truth = -np.deg2rad(3.0) * np.sin(0.5*time)
r_truth = np.deg2rad(10.0) * np.sin(0.1*time)
# r_truth = 3.0 * np.sin(1*time)

phi0 = np.deg2rad(10.0)
theta0 = np.deg2rad(10.0)

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
x_accel = sens.Accelerometer(accel_bias, accel_noise)
y_accel = sens.Accelerometer(accel_bias, accel_noise)
z_accel = sens.Accelerometer(accel_bias, accel_noise)

# Make gyroscope sensor models
gyro_bias = 1.e-3
gyro_noise = 1.e-3
x_gyro = sens.Gyroscope(gyro_bias, gyro_noise)
y_gyro = sens.Gyroscope(gyro_bias, gyro_noise)
z_gyro = sens.Gyroscope(gyro_bias, gyro_noise)

y_accel_x_truth = np.zeros(nt)
y_accel_y_truth = np.zeros(nt)
y_accel_z_truth = np.zeros(nt)

y_accel_x_meas = np.zeros(nt)
y_accel_y_meas = np.zeros(nt)
y_accel_z_meas = np.zeros(nt)

x_gyro_meas = np.zeros(nt)
y_gyro_meas = np.zeros(nt)
z_gyro_meas = np.zeros(nt)

####################################################
    #   Initialize EKF Estimation
####################################################

# random_init = np.deg2rad(3.0) * np.random.uniform((2,1))
# random_init = np.deg2rad(np.array([0.0, 5.0]))
x_hat = np.array([phi0, theta0]) # + random_init 
P_hat = np.eye(2)
Q = np.eye(2) * 1.e-4
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

    x_truth = np.array([phi, theta])
    Va = np.sqrt(u**2 + v**2 + w**2)
    # TODO: Replace 'get_x_dot' with function from kinematics.
    phi_dot, theta_dot = ekf.get_x_dot(x_truth, np.array([Va, p, q, r]))

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
    y_accel_meas = sens.get_y_accel_meas(x_accel_in, x_dot_accel_in)

    y_accel_x_truth[i] = y_accel_meas[0]
    y_accel_y_truth[i] = y_accel_meas[1]
    y_accel_z_truth[i] = y_accel_meas[2]

    y_accel_x_meas[i] = x_accel.get_sensor_data(y_accel_x_truth[i])
    y_accel_y_meas[i] = y_accel.get_sensor_data(y_accel_y_truth[i])
    y_accel_z_meas[i] = z_accel.get_sensor_data(y_accel_z_truth[i])

    # TODO: Use airspeed sensor model to get Va (and insert wind!).
    Va = np.sqrt(u**2 + v**2 + w**2)

    # Run EKF step
    y_accel_meas = np.array([y_accel_x_meas[i], y_accel_y_meas[i], y_accel_z_meas[i]])
    x_hat, P_hat = ekf.ekf_step(x_hat, P_hat, Q, R, np.array([Va, p, q, r]), y_accel_meas, dt)

    x_hat_log[i] = x_hat
    P_log[i] = P_hat

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

plt.suptitle('Sensed Acceleration vs. Measured Acceleration')

####################################################
    #   Plot Acceleration
####################################################

_, axs = plt.subplots(3,1)

axs[0].plot(time, y_accel_x_truth, label='Truth')
axs[0].plot(time, y_accel_x_meas, label='Sensor', linestyle=':')
axs[0].grid(True)
# axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('[m/s^2]')
axs[0].set_title(r'$y_{accel,x}$ vs. $y_{accel,x,meas}$')
axs[0].legend()

axs[1].plot(time, y_accel_y_truth, label='Truth')
axs[1].plot(time, y_accel_y_meas, label='Sensor', linestyle=':')
axs[1].grid(True)
# axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('[m/s^2]')
axs[1].set_title(r'$y_{accel,y}$ vs. $y_{accel,y,meas}$')
axs[1].legend()

axs[2].plot(time, y_accel_z_truth, label='Truth')
axs[2].plot(time, y_accel_z_meas, label='Sensor', linestyle=':')
axs[2].grid(True)
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('[m/s^2]')
axs[2].set_title(r'$y_{accel,z}$ vs. $y_{accel,z,meas}$')
axs[2].legend()

plt.suptitle('Attitude EKF Test')


####################################################
    #   Plot Attitude (TODO: Estimation)
####################################################

_, axs = plt.subplots(2,2)

axs[0,0].plot(time, np.rad2deg(phi_truth), label='Truth')
axs[0,0].plot(time, np.rad2deg(x_hat_log[:,0]), label='Estimation', linestyle=':')
axs[0,0].grid(True)
# axs[0,0].set_xlabel('Time [s]')
# axs[0,0].set_ylabel('[rad]')
axs[0,0].set_ylabel('[deg]')
axs[0,0].set_title(r'$\phi$ vs. $\hat{\phi}$')
axs[0,0].legend()

axs[1,0].plot(time, np.rad2deg(phi_dot_truth), label='Truth')
axs[1,0].grid(True)
axs[1,0].set_xlabel('Time [s]')
# axs[1,0].set_ylabel('[rad/s]')
axs[1,0].set_ylabel('[deg/s]')
axs[1,0].set_title(r'$\dot{\phi}$')
axs[1,0].legend()

axs[0,1].plot(time, np.rad2deg(theta_truth), label='Truth')
axs[0,1].plot(time, np.rad2deg(x_hat_log[:,1]), label='Estimation', linestyle=':')
axs[0,1].grid(True)
# axs[0,1].set_xlabel('Time [s]')
# axs[0,1].set_ylabel('[rad]')
axs[0,1].set_ylabel('[deg]')
axs[0,1].set_title(r'$\theta$ vs. $\hat{\theta}$')
axs[0,1].legend()

axs[1,1].plot(time, np.rad2deg(theta_dot_truth), label='Truth')
axs[1,1].grid(True)
axs[1,1].set_xlabel('Time [s]')
# axs[1,1].set_ylabel('[rad/s]')
axs[1,1].set_ylabel('[deg/s]')
axs[1,1].set_title(r'$\dot{\theta}$')
axs[1,1].legend()

plt.suptitle('Attitude EKF Test')

plt.show()
