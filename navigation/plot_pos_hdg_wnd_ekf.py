
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
# TODO: Consolidate initial conditions and other parameters.
from models.parameters import initial_condtions as ic
from models.parameters import vg0, chi0, wn0, we0
from simulation.kinematics import NonlinearKinematicState as nlks
from simulation.plotter import Plotter
from simulation import dynamics as dyn
import flight_controls.trim as trim
import models.sensors as sens
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

xlog_dim = [nt, len(ac_state.get_state())]

xlog = np.zeros(xlog_dim)
xlog[0,:] = ac_state.get_state()
xdot_log = np.zeros(xlog_dim)


####################################################
#   Input Forces & Moments (TODO: Add Control)
####################################################

# TODO: Use airspeed to choose trim.

Va = 30.0 # [m/s]
trim_ind = np.argmin(np.abs(trim.airspeed_trim_indeps - Va))
# print("trim_ind = ", trim_ind)
delta_e_trim = trim.elevator_trim_deps[trim_ind]
delta_t_trim = trim.throttle_trim_deps[trim_ind]
# amp =  0.1 # [N] or [N*m]
U_control = np.zeros([nt, 4])

# Step Inputs
U_control[:,0] = np.deg2rad(-4.0) * np.ones(nt) # delta_e
U_control[:,1] = 0.4 * np.ones(nt) # delta_t
# U_control[:,2] = amp * np.ones(nt) # delta_a
# U_control[:,3] = amp * np.ones(nt) # delta_r

# Sine Inputs
w_in = 2*np.pi*0.1 # [rad/s]
u_sin_wave = np.deg2rad(0.25) * np.sin(w_in * time)
U_control[:,0] = delta_e_trim
U_control[:,1] = delta_t_trim
U_control[:,2] = u_sin_wave
# U_control[:,3] = amp * u_sin_wave # delta_r


####################################################
    #   Initialize Sensor Models
####################################################

# TODO: Make GPS model (and different time-steps)
# Going to assume perfect GPS measurements, for now.

def get_gps_meas(x_ac):
    pn = x_ac[0]
    pe = x_ac[1]
    u = x_ac[3]
    v = x_ac[4]
    w = x_ac[5]
    psi = x_ac[8]
    vg = np.sqrt(u**2 + v**2 + w**2)
    vn = u * np.cos(psi) + v * np.sin(psi)
    ve = -u * np.sin(psi) + v * np.cos(psi)
    chi = np.arctan2(ve, vn)
    return np.array([pn, pe, vg, chi, 0.0, 0.0])

# TODO: Add wind to this!
def get_va_meas(x_ac):
    u = x_ac[3]
    v = x_ac[4]
    # w = x_ac[5]
    return np.sqrt(u**2 + v**2) # + w**2)

gps_log = np.zeros((nt, 6))
gps_log[0,:] = get_gps_meas(xlog[0,:])

####################################################
#   Initialize Winds
####################################################

wind_north = 0.0 # [m/s]
wind_east = 0.0 # [m/s]

# wind_north_log = np.zeros(nt)
# wind_east_log = np.zeros(nt)

# wind_north_log[0] = wind_north0
# wind_east_log[0] = wind_east0

####################################################
    #   Initialize EKF
####################################################

# TODO: get these from ekf class?
l_ekf_state = 7
m_ekf_input = 5
n_ekf_output = 6

# TODO: Don't cheat by getting initial conditions from truth data.
x_hat_pred = np.array([
    ic[0], # pn0
    ic[1], # pe0
    vg0,
    chi0,
    wn0,
    we0,
    ic[8], # psi0
])
P_pred = np.eye(l_ekf_state)
x_hat_upd = x_hat_pred.copy()
P_upd = np.eye(l_ekf_state)
Q = np.eye(l_ekf_state) * 1.e-4
R = np.eye(n_ekf_output) * 1.e-2
phw_ekf = ekf.PosHdgWndEKF(Q, R, dt)

x_hat_pred_log = np.zeros((nt, l_ekf_state))
P_pred_log = np.zeros((nt, l_ekf_state, l_ekf_state))
x_hat_upd_log = np.zeros((nt, l_ekf_state))
P_upd_log = np.zeros((nt, l_ekf_state, l_ekf_state))


####################################################
    #   Run Simulation of Sensor Models
####################################################

for i in range(nt):

    x_i = xlog[i,:]
    ac_state.set_state(x_i)
    gps_meas = get_gps_meas(x_i)

    # Run EKF step
    y_gps_meas = get_gps_meas(x_i)
    va_meas = get_va_meas(x_i)
    q_meas = x_i[9]
    r_meas = x_i[10]
    phi_meas = x_i[6]
    theta_meas = x_i[7]
    u_ekf = np.array([
        va_meas,
        q_meas,
        r_meas,
        phi_meas,
        theta_meas,])
    x_hat_pred, P_pred, x_hat_upd, P_upd = phw_ekf.ekf_step(x_hat_upd, P_upd, u_ekf, y_gps_meas)

    x_hat_pred_log[i] = x_hat_pred
    P_pred_log[i] = P_pred
    x_hat_upd_log[i] = x_hat_upd
    P_upd_log[i] = P_upd

    u_control = U_control[i,:]
    F_net, M_net = dyn.get_forces_and_moments(uav, x_i, u_control)
    xdot = ac_state.solve_f_equals_ma(uav.get_mass_props(), F_net, M_net)

    if i < (nt-1):
        x_i_plus_1 = x_i + xdot * dt # Euler integration
        ac_state.set_state(x_i_plus_1)
        xlog[i+1,:] = x_i_plus_1

    xdot_log[i,:] = xdot
    gps_log[i,:] = gps_meas


####################################################
#                   Plot Results
####################################################

fig, axs = plt.subplots(4, 2)
axs[0,0].plot(time, gps_log[:,0])
axs[0,0].plot(time, x_hat_pred_log[:,0], label='$hat{p}_n$', linestyle='--')
axs[0,0].set_title('North Position')
axs[0,0].set_ylabel('North [m]')
axs[0,0].grid(True)
axs[0,1].plot(time, gps_log[:,1])
axs[0,1].plot(time, x_hat_pred_log[:,1], label='$hat{p}_e$', linestyle='--')
axs[0,1].set_title('East Position')
axs[0,1].set_ylabel('East [m]')
axs[0,1].grid(True)
axs[1,0].plot(time, gps_log[:,2])
axs[1,0].plot(time, x_hat_pred_log[:,2], label='$hat{v}_g$')
axs[1,0].set_title('Groundspeed')
axs[1,0].set_ylabel('Groundspeed [m/s]')
axs[1,0].grid(True)
axs[1,1].plot(time, gps_log[:,3])
axs[1,1].plot(time, x_hat_pred_log[:,3], label='$hat{\chi}$')
axs[1,1].set_title('Course')
axs[1,1].set_ylabel('Course [rad]')
axs[1,1].grid(True)
axs[2,0].plot(time, gps_log[:,4])
axs[2,0].plot(time, x_hat_pred_log[:,4], label='$hat{\psi}$')
axs[2,0].set_title('Heading')
axs[2,0].set_ylabel('Heading [rad]')
axs[2,0].grid(True)
# axs[2,1].plot(time, w)
axs[2,1].plot(time, x_hat_pred_log[:,5], label='$hat{w}_n$')
axs[2,1].set_title('Wind North')
axs[2,1].set_ylabel('Wind North [m/s]')
axs[2,1].grid(True)
# axs[3,0].plot(time, gps_log[:,5])
axs[3,0].plot(time, x_hat_pred_log[:,6], label='$hat{w}_e$')
axs[3,0].set_title('Wind East')
axs[3,0].set_ylabel('Wind East [m/s]')
axs[3,0].grid(True)

# sim_plotter = Plotter(time, xlog, xdot_log)
# sim_plotter.plot_sim()

plt.show()