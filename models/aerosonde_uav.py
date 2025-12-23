
import numpy as np


###################################################
            #   Mass Properties
###################################################
m = 13.5 # [kg]
Jx = 0.8244 # [kg*m^2]
Jy = 1.135 # [kg*m^2]
Jz = 1.759 # [kg*m^2]
Jxz = 0.1204 # [kg*m^2]
# Jyz = 0.0 # [kg*m^2]
# Jxy = 0.0 # [kg*m^2]

#----------------- A/C Dimensions ----------------#

S = 0.55 # [m^2]
b = 2.8956 # [m]
c = 0.18994 # [m]

#--------------------- Engine --------------------#
# TODO: Figure out what all these are...
S_prop = 0.2027 # [m^2]
rho = 1.2682 # [kg*m^3] # TODO: Make this a function of altitude?
k_motor = 80.0 # [-]
k_T_p = 0 # [-]
k_Omega = 0.0 # [-]
e = 0.9 # [-]

###################################################
    #   Aerodynamic Coefficients [N/A]
###################################################

# TODO: Make these a lookup table?
# For A/C with wider flight envelope yes.

#----------------- Longitudinal ----------------#

C_L_0 = 0.28
C_D_0 = 0.03
C_m_0 = -0.02338

C_L_alpha = 3.45
C_D_alpha = 0.30
C_m_alpha = -0.38

C_L_q = 0.0
C_D_q = 0.0
C_m_q = -3.6

C_D_delta_e = -0.36
C_D_delta_e = 0.0
C_m_delta_e = -0.5

C_prop = 1.0
M = 50.0
alpha_0 = 0.4712
eta = 0.1592

#----------------- Lateral -----------------#

C_D_p = 0.0437

C_Y_0 = 0.0
C_l_0 = 0.0
C_n_0 = 0.0

C_Y_beta = -0.98
C_l_beta = -0.12
C_n_beta = 0.25

C_Y_p = 0.0
C_l_p = -0.26
C_n_p = 0.022

C_Y_r = 0.0
C_l_r = 0.14
C_n_r = -0.35

C_Y_delta_a = 0.0
C_l_delta_a = 0.08
C_n_delta_a = 0.06

C_Y_delta_r = -0.17
C_l_delta_r = 0.105
C_n_delta_r = -0.032

###################################################
            #   Linearized Dynamics
###################################################

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
