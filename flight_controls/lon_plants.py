
import numpy as np
import control as ct # 3rd party
import models.parameters as pm
import matplotlib.pyplot as plt
from flight_controls.utils import get_u_to_y_tf, plot_lon_step_response


####################################################
    #   Get Lon. State Space Model of A/C
####################################################

A_lon = pm.A_lon
B_lon = pm.B_lon
C_lon = np.eye(4) # ideal sensor
D_lon = np.zeros([4,2])

ss_lon = ct.ss(A_lon, B_lon, C_lon, D_lon)

####################################################
            #   Step Responses
####################################################

response = ct.step_response(ss_lon)
# response.plot() # this doesn't work...
# print(dir(response))
t = response.time

#------ Plot elevator step response -----#

x_de = response.states[:,0,:]

plot_lon_step_response(t, x_de, label="Elevator", amplitude=3.e-3)

#------ Plot throttle step response -----#

x_dt = response.states[:,1,:]

plot_lon_step_response(t, x_dt, label="Throttle")

plt.show()


####################################################
#   Functions to get Transfer Functions from SS
####################################################

# xdot = A * x + B * u
# y = C * x
# B = [B_de, B_dt]
# yi = Ci * [u, w, q, theta]^T

def get_de_to_q_tf(A_lon, B_lon, C_lon, D_lon):
    return get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 0, 2)

def get_dt_to_u_tf(A_lon, B_lon, C_lon, D_lon):
    return get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 1, 0)

def get_de_to_u_tf(A_lon, B_lon, C_lon, D_lon):
    return get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 0, 0)

def get_dt_to_q_tf(A_lon, B_lon, C_lon, D_lon):
    return get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 1, 2)


####################################################
    #   Plot A/C TFs (Bode & Nichols)
####################################################

tf_de_to_q = get_de_to_q_tf(A_lon, B_lon, C_lon, D_lon)

tf_dt_to_u = get_dt_to_u_tf(A_lon, B_lon, C_lon, D_lon)

fig, ax = plt.subplots()
ct.bode_plot(tf_de_to_q, initial_phase=0)

fig, ax = plt.subplots()
ct.bode_plot(tf_dt_to_u, initial_phase=0)

fig, ax = plt.subplots()
ct.nichols_plot(tf_de_to_q)

fig, ax = plt.subplots()
ct.nichols_plot(tf_dt_to_u)


####################################################
    #   Plot A/C TFs (Bode & Nichols)
####################################################

# tf_dt_to_q = get_dt_to_q_tf(A_lon, B_lon, C_lon, D_lon)

# tf_de_to_u = get_de_to_u_tf(A_lon, B_lon, C_lon, D_lon)

# fig, ax = plt.subplots()
# ct.bode_plot(tf_dt_to_q, initial_phase=0)

# fig, ax = plt.subplots()
# ct.bode_plot(tf_de_to_u, initial_phase=0)

# fig, ax = plt.subplots()
# ct.nichols_plot(tf_dt_to_q)

# fig, ax = plt.subplots()
# ct.nichols_plot(tf_de_to_u)


####################################################
        #   Plot A/C Poles (SP, Ph)
####################################################

ss_lon = ct.ss(A_lon, B_lon, C_lon, D_lon)

fig, ax = plt.subplots()
ct.pzmap(ss_lon)
plt.grid(True)

plt.show()
