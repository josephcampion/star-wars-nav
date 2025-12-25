
import numpy as np
import control as ct # 3rd party
import models.parameters as pm
import matplotlib.pyplot as plt
from flight_controls.utils import get_u_to_y_tf


####################################################
    #   Get Lon. State Space Model of A/C
####################################################

A_lat = pm.A_lat
B_lat = pm.B_lat
C_lat = np.eye(4) # ideal sensor
D_lat = np.zeros([4,2])


####################################################
#   Functions to get Transfer Functions from SS
####################################################

# xdot = A * x + B * u
# y = C * x
# B = [B_da, B_dr]
# yi = Ci * [v, p, phi, r]^T

def get_da_to_p_tf(A_lat, B_lat, C_lat, D_lat):
    return get_u_to_y_tf(A_lat, B_lat, C_lat, D_lat, 0, 1)

def get_dr_to_r_tf(A_lat, B_lat, C_lat, D_lat):
    return get_u_to_y_tf(A_lat, B_lat, C_lat, D_lat, 1, 3)

def get_da_to_r_tf(A_lat, B_lat, C_lat, D_lat):
    return get_u_to_y_tf(A_lat, B_lat, C_lat, D_lat, 0, 3)

def get_dr_to_p_tf(A_lat, B_lat, C_lat, D_lat):
    return get_u_to_y_tf(A_lat, B_lat, C_lat, D_lat, 1, 1)


####################################################
    #   Plot A/C TFs (Bode & Nichols)
####################################################

tf_da_to_p = get_da_to_p_tf(A_lat, B_lat, C_lat, D_lat)

tf_dr_to_r = get_dr_to_r_tf(A_lat, B_lat, C_lat, D_lat)

fig, ax = plt.subplots()
ct.bode_plot(tf_da_to_p, initial_phase=0)

fig, ax = plt.subplots()
ct.bode_plot(tf_dr_to_r, initial_phase=0)

fig, ax = plt.subplots()
ct.nichols_plot(tf_da_to_p)

fig, ax = plt.subplots()
ct.nichols_plot(tf_dr_to_r)


####################################################
    #   Plot A/C TFs (Bode & Nichols)
####################################################

# tf_da_to_r = get_da_to_r_tf(A_lat, B_lat, C_lat, D_lat)

# tf_dr_to_p = get_dr_to_p_tf(A_lat, B_lat, C_lat, D_lat)

# fig, ax = plt.subplots()
# ct.bode_plot(tf_da_to_r, initial_phase=0)

# fig, ax = plt.subplots()
# ct.bode_plot(tf_dr_to_p, initial_phase=0)

# fig, ax = plt.subplots()
# ct.nichols_plot(tf_da_to_r)

# fig, ax = plt.subplots()
# ct.nichols_plot(tf_dr_to_p)


####################################################
        #   Plot A/C Poles (DR, R, Sp)
####################################################

ss_lat = ct.ss(A_lat, B_lat, C_lat, D_lat)

fig, ax = plt.subplots()
ct.pzmap(ss_lat)
plt.grid(True)

plt.show()
