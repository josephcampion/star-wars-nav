
import numpy as np
import control
import parameters
import matplotlib.pyplot as plt

A = parameters.A_lon
B = parameters.B_lon
C = np.eye(4) # ideal sensor
D = np.zeros([4,2])

ss_sys = control.ss(A, B, C, D)

# print(ss_sys)

control.pzmap(ss_sys)
plt.show()
