
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
    #   Get Pitch Damper Gain (& Matrix)
####################################################

kp_q = 0.075
K_lon = np.zeros([2,4])
K_lon[0,2] = -kp_q
print(K_lon)

"""
NOTE: Pitch Damper has "extra" minus sign!
To pitch up, elevator deflection must go
trailing edge up (negative direction) from trim.

q_cmd = 0
de = -1.0 * kp_q * (q_cmd - q_fbk)
   = kp_q * q_fbk

u = -K * x
u = -1 * [
    0, 0, kp_q, 0;
    0, 0, 0, 0;
    ] * [u, w, q, theta]^T
"""

####################################################
#   Functions to get Transfer Functions from SS
####################################################

# xdot = A * x + B * u
# y = C * x
# B = [B_de, B_dt]
# yi = Ci * [u, w, q, theta]^T

def get_pitch_damper_tf(A_lon, B_lon, C_lon, D_lon, kp_q):

    tf_de_to_q = get_u_to_y_tf(A_lon, B_lon, C_lon, D_lon, 0, 2)

    return -kp_q * tf_de_to_q


####################################################
    #   TODO: Open-Loop Nichols
####################################################

tf_pitch_damper = get_pitch_damper_tf(A_lon, B_lon, C_lon, D_lon, kp_q)

fig, ax = plt.subplots()
ct.bode_plot(tf_pitch_damper, initial_phase=0)

fig, ax = plt.subplots()
ct.nichols_plot(tf_pitch_damper)

####################################################
        #   TODO: Step & Gust Responses
####################################################


####################################################
                # TODO: Root Locus
####################################################


####################################################
#   TODO: Plot Bare Airframe vs. Pitch Damper Poles
####################################################

ss_lon = ct.ss(A_lon, B_lon, C_lon, D_lon)

ss_pitch_damper = ct.ss(A_lon - B_lon @ K_lon, np.zeros([4,2]), C_lon, D_lon)

fig, ax = plt.subplots()
ct.pzmap(ss_lon)
plt.grid(True)

fig, ax = plt.subplots()
ct.pzmap(ss_pitch_damper)
plt.grid(True)

plt.show()
