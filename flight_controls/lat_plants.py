
import numpy as np
import control as ct # 3rd party
import models.parameters as pm
import matplotlib.pyplot as plt


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

def get_da_to_p_tf(A_lat, B_lat, C_lat, D_lat):

    # B = [B_da, B_dr]
    # shape needs to be (4,1), not (4,)
    B_da = np.array([[B_lat[i,0]] for i in range(4)])

    # y = C * x
    # p = [0 1 0 0] * [v p phi r]^T
    C_p = np.zeros([1,4])
    C_p[0,1] = 1

    D_p_da = np.array([[0.0]]) # shape needs to be (1,1)

    tf_da_to_p = ct.ss2tf(ct.ss(A_lat, B_da, C_p, D_p_da))

    return tf_da_to_p


def get_dr_to_r_tf(A_lat, B_lat, C_lat, D_lat):

    # B = [B_da, B_dr]
    # shape needs to be (4,1), not (4,)
    B_dr = np.array([[B_lat[i,1]] for i in range(4)])

    # y = C * x
    # r = [0 0 0 1] * [v p phi r]^T
    C_r = np.zeros([1,4])
    C_r[0,3] = 1

    D_r_dr = np.array([[0.0]]) # shape needs to be (1,1)

    tf_dr_to_r = ct.ss2tf(ct.ss(A_lat, B_dr, C_r, D_r_dr))

    return tf_dr_to_r


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
        #   Plot A/C Poles (DR, R, Sp)
####################################################

ss_lat = ct.ss(A_lat, B_lat, C_lat, D_lat)

fig, ax = plt.subplots()
ct.pzmap(ss_lat)
plt.grid(True)

plt.show()
