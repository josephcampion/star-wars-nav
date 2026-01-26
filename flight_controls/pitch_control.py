
import numpy as np
import control as ct # 3rd party
import matplotlib.pyplot as plt
import flight_controls.transfer_functions as tfs
import flight_controls.utils as utils
from flight_controls.utils import mag2db
from models.actuator import tf_actuator
import flight_controls.gains as gains


####################################################
#   Create Pitch Rate Inner Loop Transfer Function
####################################################

tf_de_to_q = tfs.tf_de_to_q 
tf_de_cmd_to_q = tf_de_to_q * tf_actuator

s = ct.tf('s')
tf_q_ctrl = -1.0 * (gains.kp_q + gains.ki_q / s)
# print(tf_q_ctrl)

tf_q_closed_loop = (tf_q_ctrl * tf_de_cmd_to_q) / (1 + tf_q_ctrl * tf_de_cmd_to_q)
# print(tf_q_closed_loop)


####################################################
                # TODO: Root Locus
####################################################


####################################################
        #   Pitch Controller
####################################################

s = ct.tf('s')
tf_theta_ctrl = gains.kp_theta + gains.ki_theta / s # + gains.kd_theta * s
# print("tf_theta_ctrl = ", tf_theta_ctrl)

tf_theta_open_loop = tf_theta_ctrl * tf_q_closed_loop
# print("tf_theta_open_loop = ", tf_theta_open_loop)

tf_theta_closed_loop = (tf_theta_ctrl * tf_q_closed_loop) / (1 + tf_theta_ctrl * tf_q_closed_loop)
# print("tf_theta_closed_loop = ", tf_theta_closed_loop)


####################################################
# Plot open-loop Nichols for stability analysis
####################################################

fig, ax = plt.subplots()
mag, phase_rad, omega = ct.bode(tf_theta_open_loop, plot=False)
phase_deg = np.rad2deg(phase_rad)
phase_deg = utils.wrap_phase(phase_deg)
utils.plot_nichols(mag2db(mag), phase_deg, ax, label="Pitch Open-Loop")
utils.plot_nichols_margins(ax)
ax.set_title(r'$G_{ol}(s)=\frac{\theta(s)}{e_{\theta}(s)}=K_\theta(s)\cdot P_\theta(s)$')


####################################################
# Plot OPEN-loop Bode for control design
####################################################

fig, ax = plt.subplots(2,1)
mag, phase_rad, omega = ct.bode(tf_theta_open_loop, plot=False)
phase_deg = np.rad2deg(phase_rad)
phase_deg = utils.wrap_phase(phase_deg)
utils.plot_bode(mag2db(mag), phase_deg, omega, ax,label="Pitch Open-Loop")
fig.suptitle(r'$G_{ol}(s)=\frac{\theta(s)}{e_{\theta}(s)}$')


####################################################
# Plot CLOSED-loop Bode for performance analysis
####################################################

fig, ax = plt.subplots(2,1)
mag, phase_rad, omega = ct.bode(tf_theta_closed_loop, plot=False)
phase_deg = np.rad2deg(phase_rad)
phase_deg = utils.wrap_phase(phase_deg)
utils.plot_bode(mag2db(mag), phase_deg, omega, ax,label="Pitch Closed-Loop")
fig.suptitle(r'$G_{cl}(s)=\frac{\theta(s)}{\theta_{cmd}(s)}$')


####################################################
#   Closed-Loop Step Response
#################################################### 

dt = 1.e-3
Tsim = 10.0
timepts = np.arange(0, Tsim, dt)
U = np.deg2rad(10.0) * np.ones([1,len(timepts)])
response = ct.forced_response(tf_theta_closed_loop, timepts, U)
t = response.time
p_t = response.y[0,:]

fig, ax = plt.subplots()
ax.plot(t, np.rad2deg(p_t), label="Pitch Closed Loop")
ax.set_title(r'$G_{cl}(s)=\frac{\theta(s)}{\theta_{cmd}(s)}$')
ax.set_ylabel('[deg/s]')
ax.set_xlabel('Time [s]')
ax.grid(True)


# Plot all the things!
plt.show()
