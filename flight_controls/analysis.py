
import numpy as np
import control as ct # 3rd party
import flight_controls.dynamics as dyn
import matplotlib.pyplot as plt

# def my_bode_plot(frd):


####################################################
    #   Run Simulation of Sensor Models
####################################################

A_lon = dyn.A_lon
B_lon = dyn.B_lon
print(B_lon)
C_lon = np.eye(4) # ideal sensor
D_lon = np.zeros([4,2])

# Aircraft State Space
ss_ac_lon = ct.ss(A_lon, B_lon, C_lon, D_lon)

# tf_ac_lon = ct.ss2tf(ss_ac_lon)
# print(tf_ac_lon)

B_de = np.array([[B_lon[i,0]] for i in range(4)])
print(B_de)

print("np.shape(B_de) = ", np.shape(B_de))
C_q = np.zeros([1,4])
C_q[0,2] = 1
print(C_q)
print("np.shape(C_q) = ", np.shape(C_q))
D_q_de = np.array([[0.0]])
print("np.shape(D_q_de) = ", np.shape(D_q_de))

tf_de_to_q = ct.ss2tf(ct.ss(A_lon, B_de, C_q, D_q_de))
print("tf_de_to_q = ", tf_de_to_q)
ct.bode_plot(tf_de_to_q, initial_phase=0)

# ct.pzmap(ss_ac_lon)
# plt.grid(True)
plt.show()
