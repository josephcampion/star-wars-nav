
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
from models.aerodynamics import AerodynamicCoefficients

# C_Y_0, C_Y_p, C_Y_r, and C_Y_delta_a are zero.
# So the interesting plots will be with C_Y_beta and C_Y_delta_r.

# TODO: Sweep airspeed range and density.
Va = 20.0 # [m/s]
rho = 1.2682 # [kg/m^3]
p = np.deg2rad(0.0) # [rad/s]
r = np.deg2rad(0.0) # [rad/s]
delta_a = 0.0 # [rad]
delta_r = 0.0 # [rad]

####################################################
        #   Initialize UAV Parameters
####################################################

aero_coeffs = AerodynamicCoefficients([
    mav.C_L_0, mav.C_D_0, mav.C_m_0, \
    mav.C_L_alpha, mav.C_D_alpha, mav.C_m_alpha, \
    mav.C_L_q, mav.C_D_q, mav.C_m_q, \
    mav.C_L_delta_e, mav.C_D_delta_e, mav.C_m_delta_e, \
    mav.C_Y_0, mav.C_l_0, mav.C_n_0, \
    mav.C_Y_beta, mav.C_l_beta, mav.C_n_beta, \
    mav.C_Y_p, mav.C_l_p, mav.C_n_p, \
    mav.C_Y_r, mav.C_l_r, mav.C_n_r, \
    mav.C_Y_delta_a, mav.C_l_delta_a, mav.C_n_delta_a, \
    mav.C_Y_delta_r, mav.C_l_delta_r, mav.C_n_delta_r \
])

####################################################
#   Sweep angle of sideslip for get side force.
####################################################

n_beta_sweep = 50
beta_sweep_deg = np.linspace(-10.0, 10.0, n_beta_sweep)
beta_sweep_rad = np.deg2rad(beta_sweep_deg)

F_side_beta_sweep = np.zeros(len(beta_sweep_rad))

for i, beta in enumerate(beta_sweep_rad):
    F_side_beta_sweep[i] = aero_coeffs.get_side_force(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)

####################################################
#   Sweep rudder deflection to get side force
####################################################

beta = np.deg2rad(0.0) # [rad]

n_delta_r_sweep = 50
delta_r_sweep_deg = np.linspace(-10.0, 10.0, n_delta_r_sweep)
delta_r_sweep_rad = np.deg2rad(delta_r_sweep_deg)

F_side_delta_r_sweep = np.zeros(len(delta_r_sweep_rad))

for i, delta_r in enumerate(delta_r_sweep_rad):
    F_side_delta_r_sweep[i] = aero_coeffs.get_side_force(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)

####################################################
            #   Plot Results
####################################################

_, axs = plt.subplots(2,1)

axs[0].plot(beta_sweep_deg, F_side_beta_sweep)
axs[0].grid(True)
axs[0].set_xlabel('Beta [deg]')
axs[0].set_ylabel('[N]')
axs[0].set_title('Side Force vs. Beta')

axs[1].plot(delta_r_sweep_deg, F_side_delta_r_sweep)
axs[1].grid(True)
axs[1].set_xlabel('Rudder Deflection [deg]')
axs[1].set_ylabel('[N]')
axs[1].set_title('Side Force vs. Rudder Deflection')

plt.show()
