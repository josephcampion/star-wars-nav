
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
# TODO: Consolidate initial conditions and other parameters.
from models.parameters import initial_condtions as ic
from models.parameters import vg0, chi0, wn0, we0
from simulation.kinematics import NonlinearKinematicState as nlks
from simulation.plotter import Plotter
from simulation import dynamics as dyn

import models.sensors as sens
# import simulation.rotations as rt
import navigation.pos_hdg_wnd_ekf as ekf

####################################################
#   Initialize Vehicle and Simulation
####################################################

uav = mav.vehicle

Tsim = 60.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

# This can get figured out in Simulation class:
time = np.arange(t0, Tsim, dt)
nt = len(time)

# Initialize vectors 
ac_state = nlks(init_conds=ic)

xdim = [nt, len(ac_state.get_state())]

X = np.zeros(xdim)
X[0,:] = ac_state.get_state()
Xdot = np.zeros(xdim)


####################################################
#   Input Forces & Moments (TODO: Add Control)
####################################################

# TODO: Add trim inputs to this (for elevator and throttle).

# amp =  0.1 # [N] or [N*m]
U_control = np.zeros([nt, 4])

# Step Inputs
U_control[:,0] = np.deg2rad(-4.0) * np.ones(nt) # delta_e
U_control[:,1] = 0.4 * np.ones(nt) # delta_t
# U_control[:,2] = amp * np.ones(nt) # delta_a
# U_control[:,3] = amp * np.ones(nt) # delta_r

# Sine Inputs
w_in = 1.0 # [rad/s]
# u_sin_wave = 0.1 * np.sin(w_in * time)
# U_control[:,0] = u_sin_wave # delta_e
# U_control[:,1] = amp * -u_sin_wave # delta_t
# U_control[:,2] = amp * u_sin_wave # delta_a
# U_control[:,3] = amp * u_sin_wave # delta_r


####################################################
    #   Initialize Sensor Models
####################################################

# TODO: Make GPS model (and different time-steps)

####################################################
    #   Initialize EKF Estimation
####################################################

# TODO: get these from ekf class?
l_ekf_state = 7
m_ekf_input = 5
n_ekf_output = 6

# TODO: Don't cheat by getting initial conditions from truth data.
x_hat_pred = np.array([
    ic[0],
    ic[1],
    vg0,
    chi0,
    wn0,
    we0,
    ic[8]])
P_pred = np.eye(l_ekf_state)
x_hat_upd = x_hat_pred.copy()
P_upd = np.eye(l_ekf_state)
Q = np.eye(l_ekf_state) * 1.e-4
R = np.eye(6) * 1.e-2
pr_ekf = ekf.PosHdgWndEKF(Q, R, dt)

x_hat_pred_log = np.zeros((nt, l_ekf_state))
P_pred_log = np.zeros((nt, l_ekf_state, l_ekf_state))
x_hat_upd_log = np.zeros((nt, l_ekf_state))
P_upd_log = np.zeros((nt, l_ekf_state, l_ekf_state))


####################################################
    #   Run Simulation of Sensor Models
####################################################

for i in range(nt):

    x = X[i,:]
    ac_state.set_state(x)

    u_control = U_control[i,:]

    F_net, M_net = dyn.get_forces_and_moments(uav, x, u_control)

    xdot = ac_state.solve_f_equals_ma(uav.get_mass_props(), F_net, M_net)

    Xdot[i,:] = xdot

    """
    # Insert truth motion into sensor models
    # x = [u, v, w, phi, theta, p, q, r]
    # xdot = [udot, vdot, wdot]
    x_accel_in = np.array([u, v, w, phi, theta, p, q, r])
    # TODO: Insert x_dot_accel_in from kinematics.
    x_dot_accel_in = np.zeros(3) # np.array([r])

    # Run EKF step
    y_accel_meas = np.array([y_accel_x_meas[i], y_accel_y_meas[i], y_accel_z_meas[i]])
    # x_hat_pred, P_pred, x_hat_upd, P_upd = pr_ekf.ekf_step(x_hat_pred, P_pred, np.array([Va, p, q, r]), y_accel_meas)
    x_hat_pred, P_pred, x_hat_upd, P_upd = pr_ekf.ekf_step(x_hat_upd, P_upd, np.array([Va, p, q, r]), y_accel_meas)

    x_hat_pred_log[i] = x_hat_pred
    P_pred_log[i] = P_pred
    x_hat_upd_log[i] = x_hat_upd
    P_upd_log[i] = P_upd
    """

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt
        ac_state.set_state(X[i+1,:])

####################################################
#                   Plot Results
####################################################

sim_plotter = Plotter(time, X, Xdot)
sim_plotter.plot_sim()
