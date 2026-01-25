
import numpy as np
import control as ct # 3rd party
import matplotlib.pyplot as plt
import flight_controls.transfer_functions as tfs


tf_roll = tfs.tf_da_to_roll

# ----- Plot Bode & Nichols ----- #

fig, ax = plt.subplots()
ct.bode_plot(tf_roll, initial_phase=0)
ax.grid(True)

fig, ax = plt.subplots()
ct.nichols_plot(tf_roll)
ax.grid(True)


####################################################
            #   Step Response
####################################################

response = ct.step_response(tf_roll, T=1.0)
t = response.time
roll_t = response.x[0,0,:] # roll response

#------ Plot Bare Airframe Response -----#

fig, ax = plt.subplots()

ax.plot(t, roll_t, label="Roll")
ax.set_title(r'$\phi$')
ax.set_ylabel('[rad]')
ax.set_xlabel('Time [s]')
ax.grid(True)

#------- Compare to Roll Control --------#

####################################################
                # TODO: Root Locus
####################################################


####################################################
        #   Make Roll Controller
####################################################

kp_phi = 1.e2
ki_phi = 0.0
kd_phi = 0.0

# TODO: Need minus sign because aileron deflection is opposite of induced roll rate.
tf_roll_ctrl = ct.tf([-kp_phi], [1.0])
print(tf_roll_ctrl)
tf_roll_closed_loop = (tf_roll_ctrl * tf_roll) / (1 + tf_roll_ctrl * tf_roll)
print(tf_roll_closed_loop)

# ----- Plot Bode & Nichols ----- #

fig, ax = plt.subplots()
ct.bode_plot(tf_roll_closed_loop, initial_phase=0)
ax.grid(True)

fig, ax = plt.subplots()
ct.nichols_plot(tf_roll_closed_loop)
ax.grid(True)


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
