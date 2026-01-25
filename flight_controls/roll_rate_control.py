
import numpy as np
import control as ct # 3rd party
import matplotlib.pyplot as plt
import flight_controls.transfer_functions as tfs
import flight_controls.utils as utils
from flight_controls.utils import mag2db
from models.actuator import tf_actuator

# Include actuator dynamics in open-loop transfer function.
tf_da_to_p = tfs.tf_da_to_p 
tf_da_cmd_to_p = tf_da_to_p * tf_actuator

####################################################
            #   Bode Plots
####################################################

fig, ax = plt.subplots(2,1)

mag, phase_rad, omega = ct.bode(tf_da_to_p, plot=False)
utils.plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega, ax,label="Aileron to Roll Rate")

mag, phase_rad, omega = ct.bode(tf_actuator, plot=False)
utils.plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega, ax, label="Actuator Loop")

mag, phase_rad, omega = ct.bode(tf_da_cmd_to_p, plot=False)
utils.plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega, ax, label="Open-Loop")

fig.suptitle(r'$P(s)=\frac{p(s)}{\delta_{a,cmd}(s)}$')

####################################################
            #   Nichols Plots
####################################################


fig, ax = plt.subplots()

mag, phase_rad, omega = ct.bode(tf_da_to_p, plot=False)
utils.plot_nichols(mag2db(mag), np.rad2deg(phase_rad), ax, label="Aileron to Roll Rate")

mag, phase_rad, omega = ct.bode(tf_actuator, plot=False)
utils.plot_nichols(mag2db(mag), np.rad2deg(phase_rad), ax, label="Actuator Loop")

mag, phase_rad, omega = ct.bode(tf_da_cmd_to_p, plot=False)
utils.plot_nichols(mag2db(mag), np.rad2deg(phase_rad), ax, label="Open-Loop")

gm = 8.0 # [dB]
pm = 45.0 # [deg]
utils.plot_nichols_margins(ax)

ax.set_title(r'$P(s)=\frac{p(s)}{\delta_{a,cmd}(s)}$')

####################################################
            #   Step Response
####################################################

dt = 0.001
Tsim = 1.0
time_pts = np.arange(0, Tsim, dt)
response = ct.step_response(tf_da_cmd_to_p, T=Tsim, T_num=1000)
t = response.time
p_t = response.y[0,0,:] # roll rate response

#------ Plot Bare Airframe Response -----#

fig, ax = plt.subplots()

ax.plot(t, np.rad2deg(p_t), label="Roll Rate")
ax.set_title(r'$P(s)=\frac{p(s)}{\delta_{a,cmd}(s)}$')
ax.set_ylabel('[deg/s]')
ax.set_xlabel('Time [s]')
ax.grid(True)

#------- Compare to Roll Control --------#

####################################################
                # TODO: Root Locus
####################################################


####################################################
        #   Roll Rate Controller
####################################################

kp_p = 1.e2
ki_p = 0.0
# kd_p = 0.0

# TODO: Need minus sign because aileron deflection is opposite of induced roll rate.
# tf_roll_ctrl = ct.tf([-kp_phi], [1.0])
# print(tf_roll_ctrl)
# tf_roll_closed_loop = (tf_roll_ctrl * tf_roll) / (1 + tf_roll_ctrl * tf_roll)
# print(tf_roll_closed_loop)

# ----- Plot Bode & Nichols ----- #

# fig, ax = plt.subplots()
# ct.bode_plot(tf_roll_closed_loop, initial_phase=0)
# ax.grid(True)

# fig, ax = plt.subplots()
# ct.nichols_plot(tf_roll_closed_loop)
# ax.grid(True)


####################################################
            #   Step Response
####################################################

# response = ct.step_response(tf_roll_closed_loop, T=5.0)
# t = response.time
# # print(response.x)
# roll_t = response.x[2,0,:] # roll response

# #------ Plot Bare Airframe Response -----#

# fig, ax = plt.subplots()

# ax.plot(t, roll_t, label="Roll Closed Loop")
# ax.set_title(r'$\phi$')
# ax.set_ylabel('[rad]')
# ax.set_xlabel('Time [s]')
# ax.grid(True)

####################################################
#   TODO: Plot Bare Airframe vs. Pitch Damper Poles
####################################################

# ss_lon = ct.ss(A_lon, B_lon, C_lon, D_lon)

# ss_pitch_damper = ct.ss(A_lon - B_lon @ K_lon, np.zeros([4,2]), C_lon, D_lon)

# fig, ax = plt.subplots()
# ct.pzmap(ss_lon)
# plt.grid(True)

# fig, ax = plt.subplots()
# ct.pzmap(ss_pitch_damper)
# plt.grid(True)

plt.show()
