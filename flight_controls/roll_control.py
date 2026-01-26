
import numpy as np
import control as ct # 3rd party
import matplotlib.pyplot as plt
import flight_controls.transfer_functions as tfs
import flight_controls.utils as utils
from flight_controls.utils import mag2db
from models.actuator import tf_actuator
import flight_controls.gains as gains


####################################################
#   Create Roll Rate Inner Loop Transfer Function
####################################################

# Include actuator dynamics in open-loop transfer function.
tf_da_to_p = tfs.tf_da_to_p 
tf_da_cmd_to_p = tf_da_to_p * tf_actuator

s = ct.tf('s')
tf_p_ctrl = gains.kp_p + gains.ki_p / s
# print(tf_p_ctrl)

tf_p_closed_loop = (tf_p_ctrl * tf_da_cmd_to_p) / (1 + tf_p_ctrl * tf_da_cmd_to_p)
# print(tf_p_closed_loop)


####################################################
                # TODO: Root Locus
####################################################


####################################################
        #   Roll Controller
####################################################

s = ct.tf('s')
tf_phi_ctrl = gains.kp_phi + gains.ki_phi / s + gains.kd_phi * s
print(tf_phi_ctrl)

tf_phi_open_loop = tf_phi_ctrl * tf_p_closed_loop

tf_phi_closed_loop = (tf_phi_ctrl * tf_p_closed_loop) / (1 + tf_phi_ctrl * tf_p_closed_loop)
# print(tf_p_closed_loop)


####################################################
# Plot open-loop Nichols for stability analysis
####################################################

fig, ax = plt.subplots()
mag, phase_rad, omega = ct.bode(tf_phi_open_loop, plot=False)
utils.plot_nichols(mag2db(mag), np.rad2deg(phase_rad), ax, label="Roll Open-Loop")
utils.plot_nichols_margins(ax)
ax.set_title(r'$G_{ol}(s)=\frac{\phi(s)}{e_{\phi}(s)}=K_\phi(s)\cdot P_\phi(s)$')


####################################################
# Plot OPEN-loop Bode for control design
####################################################

fig, ax = plt.subplots(2,1)
mag, phase_rad, omega = ct.bode(tf_phi_open_loop, plot=False)
utils.plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega, ax,label="Roll Open-Loop")
utils.plot_bode_margins(ax, omega[0], omega[-1])
fig.suptitle(r'$G_{ol}(s)=\frac{\phi(s)}{e_{\phi}(s)}$')


####################################################
# Plot CLOSED-loop Bode for performance analysis
####################################################

fig, ax = plt.subplots(2,1)
mag, phase_rad, omega = ct.bode(tf_phi_closed_loop, plot=False)
utils.plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega, ax,label="Roll Closed-Loop")
fig.suptitle(r'$G_{cl}(s)=\frac{\phi(s)}{\phi_{cmd}(s)}$')


####################################################
#   Closed-Loop Step Response
####################################################

dt = 1.e-3
Tsim = 1.0
timepts = np.arange(0, Tsim, dt)
U = np.deg2rad(10.0) * np.ones([1,len(timepts)])
response = ct.forced_response(tf_phi_closed_loop, timepts, U)
t = response.time
p_t = response.y[0,:]

fig, ax = plt.subplots()
ax.plot(t, np.rad2deg(p_t), label="Roll Closed Loop")
ax.set_title(r'$G_{cl}(s)=\frac{\phi(s)}{\phi_{cmd}(s)}$')
ax.set_ylabel('[deg/s]')
ax.set_xlabel('Time [s]')
ax.grid(True)


# Plot all the things!
plt.show()
