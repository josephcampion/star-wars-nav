

import numpy as np
import matplotlib.pyplot as plt

import simulation.kinematics as kin

####################################################
    #   Kinematics and Accelerometer
####################################################

# x = [roll, pitch]^T = [phi, theta]^T
def get_x_dot(p, q, r, phi, theta):
    sp = np.sin(phi)
    cp = np.cos(phi)
    tt = np.tan(theta)
    phi_dot = p + q * sp * tt + r * cp * tt
    theta_dot = q * cp - r * sp
    return np.array([phi_dot, theta_dot])

"""
x = [
pn, pe, pd,
u, v, w,
phi, theta, psi,
p, q, r]

xdot = [
pn_dot, pe_dot, pd_dot,
udot, vdot, wdot,
phi_dot, theta_dot, psi_dot,
pdot, qdot, rdot]
"""
def get_y_accel(x, xdot, noise=np.zeros(3)):

    u, v, w = x[3:6]
    phi, theta, _ = x[6:9]
    p, q, r = x[9:12]

    udot, vdot, wdot = xdot[3:6]

    g = 9.81

    y_accel_x  = udot + q*w - r*v + g*np.sin(theta) # + noise[0]
    y_accel_y = vdot + r*u - p*w - g*np.cos(theta)*np.sin(phi) # + noise[1]
    y_accel_z = wdot + p*v - q*u - g*np.cos(theta)*np.cos(phi) # + noise[2]

    return np.array([y_accel_x, y_accel_y, y_accel_z])


####################################################
    #   Initialize Simulation Parameters
####################################################

Tsim = 1.0 # seconds
dt = 0.001 # timestep
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
    #   Run Simulation of Sensor Models
####################################################

for i in range(nt-1):

    phi = phi_truth[i]
    theta = theta_truth[i]

    u = u_truth[i]
    v = v_truth[i]
    w = w_truth[i]

    p = p_truth[i]
    q = q_truth[i]
    r = r_truth[i]

    phi_dot, theta_dot = get_x_dot(p, q, r, phi, theta)

    phi_dot_truth[i] = phi_dot
    theta_dot_truth[i] = theta_dot

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
axs[0,1].grid(True)
# axs[0,1].set_xlabel('Time [s]')
axs[0,1].set_ylabel('[rad/s]')
axs[0,1].set_title(r'$p$')
axs[0,1].legend()

axs[1,1].plot(time, q_truth, label='Truth')
axs[1,1].grid(True)
# axs[1,1].set_xlabel('Time [s]')
axs[1,1].set_ylabel('[rad/s]')
axs[1,1].set_title(r'$q$')
axs[1,1].legend()

axs[2,1].plot(time, r_truth, label='Truth')
axs[2,1].grid(True)
axs[2,1].set_xlabel('Time [s]')
axs[2,1].set_ylabel('[rad/s]')
axs[2,1].set_title(r'$r$')
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
