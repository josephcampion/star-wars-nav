
import numpy as np

# TODO: Figure out how to consolidate this with 'models/paramters.py'

######################################################################################
                #   Linearized Dynamics
######################################################################################

g = 9.81 # [m/s^2]

# Longitudinal state-space matrix components
X_u = -0.03
X_w = 0.02
X_q = 0.0

Z_u = -0.2
Z_w = -1.0
Z_q = 50.0

M_u = 0.001
M_w = -0.02
M_q = -1.5

X_de = 0.0
X_dt = 0.5
Z_de = -3.0
M_de = -20.0

A_lon = np.array([
    [X_u, X_w, X_q, -g],
    [Z_u, Z_w, Z_q, 0.0],
    [M_u, M_w, M_q, 0.0],
    [0.0, 0.0, 1.0, 0.0]
])

B_lon = np.array([
    [X_de, X_dt],
    [Z_de, 0.0],
    [M_de, 0.0],
    [0.0, 0.0]
])

# Lateral state-space matrix components
Y_v = -0.5
Y_p = 0.0
Y_r = 1.0

L_v = -0.1
L_p = -4.0 # roll damping
L_r = 0.7 # di-hedral effect?

N_v = 0.05
N_p = -0.5 # adverse yaw?
N_r = -0.3 # yaw damping

Y_da = 0.0
Y_dr = 0.2
L_da = 10.0
L_dr = 1.0
N_da = 0.5
N_dr = -5.0

A_lat = np.array([
    [Y_v, Y_p, Y_r, g],
    [L_v, L_p, L_r, 0.0],
    [N_v, N_p, N_r, 0.0],
    [0.0, 1.0, 0.0, 0.0]
])

B_lat = np.array([
    [Y_da, Y_dr],
    [L_da, L_dr],
    [N_da, N_dr],
    [0.0, 0.0]
])

# B_lat_da = np.array([
#     [Y_da],
#     [L_da],
#     [N_da],
#     [0.0]
# ])

# B_lat_da = np.array([[B_lat[i,0]] for i in range(4)])

