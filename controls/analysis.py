
import numpy as np
import control # 3rd party
import dynamics as dyn
import matplotlib.pyplot as plt

A = dyn.A_lon
B = dyn.B_lon
C = np.eye(4) # ideal sensor
D = np.zeros([4,2])

ss_sys = control.ss(A, B, C, D)

# print(ss_sys)

control.pzmap(ss_sys)
plt.show()
