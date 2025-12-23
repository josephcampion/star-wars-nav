
import numpy as np
import control as ct # 3rd party
import models.parameters as pm
import matplotlib.pyplot as plt
from flight_controls.utils import get_u_to_y_tf


####################################################
    #   Get Lon. State Space Model of A/C
####################################################

A_lon = pm.A_lon
B_lon = pm.B_lon
C_lon = np.eye(4) # ideal sensor
D_lon = np.zeros([4,2])


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

def get_dt_to_q_tf(A_lon, B_lon, C_lon, D_lon):
    return get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 1, 2)

def get_de_to_u_tf(A_lon, B_lon, C_lon, D_lon):
    return get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 0, 0)


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

# ss_lat = ct.ss(A_lat, B_lat, C_lat, D_lat)

fig, ax = plt.subplots()
ct.pzmap(ss_lon)
plt.grid(True)

# fig, ax = plt.subplots()
# ct.pzmap(ss_lat)
# plt.grid(True)

plt.show()
