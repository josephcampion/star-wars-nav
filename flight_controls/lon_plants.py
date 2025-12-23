
import numpy as np
import control as ct # 3rd party
import models.parameters as pm
import matplotlib.pyplot as plt


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

def get_de_to_q_tf(A_lon, B_lon, C_lon, D_lon):

    # B = [B_de, B_dt]
    # shape needs to be (4,1), not (4,)
    B_de = np.array([[B_lon[i,0]] for i in range(4)])

    # y = C * x
    # q = [0 0 1 0] * [u w q theta]^T
    C_q = np.zeros([1,4])
    C_q[0,2] = 1

    D_q_de = np.array([[0.0]]) # shape needs to be (1,1)

    tf_de_to_q = ct.ss2tf(ct.ss(A_lon, B_de, C_q, D_q_de))

    return tf_de_to_q


def get_dt_to_u_tf(A_lon, B_lon, C_lon, D_lon):

    # B = [B_de, B_dt]
    # shape needs to be (4,1), not (4,)
    B_dt = np.array([[B_lon[i,1]] for i in range(4)])

    # y = C * x
    # u = [1 0 0 0] * [u w q theta]^T
    C_u = np.zeros([1,4])
    C_u[0,0] = 1

    D_u_dt = np.array([[0.0]]) # shape needs to be (1,1)

    tf_dt_to_u = ct.ss2tf(ct.ss(A_lon, B_dt, C_u, D_u_dt))

    return tf_dt_to_u


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
