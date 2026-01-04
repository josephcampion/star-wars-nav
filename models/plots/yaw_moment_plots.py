
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
from models.aerodynamics import AerodynamicCoefficients

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

# TODO: Sweep airspeed range and density.
Va = 20.0 # [m/s]
rho = 1.2682 # [kg/m^3]

####################################################
#   Sweep angle of sideslip vs. roll moment.
####################################################

# beta = np.deg2rad(0.0) # [rad]
p = np.deg2rad(0.0) # [rad/s]
r = np.deg2rad(0.0) # [rad/s]
delta_a = 0.0 # [rad]
delta_r = 0.0 # [rad]

n_beta_sweep = 50
beta_sweep_deg = np.linspace(-10.0, 10.0, n_beta_sweep)
beta_sweep_rad = np.deg2rad(beta_sweep_deg)

M_yaw_beta_sweep = np.zeros(len(beta_sweep_rad))

for i, beta in enumerate(beta_sweep_rad):
    M_yaw_beta_sweep[i] = aero_coeffs.get_yaw_moment(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)

####################################################
#   Sweep roll rate to get roll moment.
####################################################

beta = np.deg2rad(0.0) # [rad]
# p = np.deg2rad(0.0) # [rad/s]
r = np.deg2rad(0.0) # [rad/s]
delta_a = 0.0 # [rad]
delta_r = 0.0 # [rad]

n_p_sweep = 50
p_sweep_dps = np.linspace(-10.0, 10.0, n_p_sweep) # [deg/s]
p_sweep_rps = np.deg2rad(p_sweep_dps) # [rad/s]

M_yaw_p_sweep = np.zeros(len(p_sweep_rps))

for i, p in enumerate(p_sweep_rps):
    M_yaw_p_sweep[i] = aero_coeffs.get_yaw_moment(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)

####################################################
#   Sweep yaw rate to get roll moment.
####################################################

beta = np.deg2rad(0.0) # [rad]
p = np.deg2rad(0.0) # [rad/s]
# r = np.deg2rad(0.0) # [rad/s]
delta_a = 0.0 # [rad]
delta_r = 0.0 # [rad]

n_r_sweep = 50
r_sweep_dps = np.linspace(-5.0, 5.0, n_r_sweep) # [deg/s]
r_sweep_rps = np.deg2rad(r_sweep_dps) # [rad/s]

M_yaw_r_sweep = np.zeros(len(r_sweep_rps))

for i, r in enumerate(r_sweep_rps):
    M_yaw_r_sweep[i] = aero_coeffs.get_yaw_moment(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)

####################################################
#   Sweep aileron deflection to get roll moment.
####################################################

beta = np.deg2rad(0.0) # [rad]
p = np.deg2rad(0.0) # [rad/s]
r = np.deg2rad(0.0) # [rad/s]
# delta_a = 0.0 # [rad]
delta_r = 0.0 # [rad]

n_delta_a_sweep = 50
delta_a_sweep_deg = np.linspace(-10.0, 10.0, n_delta_a_sweep)
delta_a_sweep_rad = np.deg2rad(delta_a_sweep_deg)

M_yaw_delta_a_sweep = np.zeros(len(delta_a_sweep_rad))

for i, delta_a in enumerate(delta_a_sweep_rad):
    M_yaw_delta_a_sweep[i] = aero_coeffs.get_yaw_moment(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)

####################################################
#   Sweep rudder deflection to get roll moment.
####################################################

beta = np.deg2rad(0.0) # [rad]
p = np.deg2rad(0.0) # [rad/s]
r = np.deg2rad(0.0) # [rad/s]
delta_a = 0.0 # [rad]
# delta_r = 0.0 # [rad]

n_delta_r_sweep = 50
delta_r_sweep_deg = np.linspace(-10.0, 10.0, n_delta_r_sweep)
delta_r_sweep_rad = np.deg2rad(delta_r_sweep_deg)

M_yaw_delta_r_sweep = np.zeros(len(delta_r_sweep_rad))

for i, delta_r in enumerate(delta_r_sweep_rad):
    M_yaw_delta_r_sweep[i] = aero_coeffs.get_yaw_moment(beta, p, r, delta_a, delta_r, rho, Va, mav.S, mav.b)


####################################################
            #   Plot Results
####################################################

fig, axs = plt.subplots(3,2)

axs[0,0].plot(beta_sweep_deg, M_yaw_beta_sweep)
axs[0,0].grid(True)
axs[0,0].set_xlabel('Beta [deg]')
axs[0,0].set_ylabel(r'$M_z$ [N*m]')
# axs[0].set_title('Yaw Moment vs. Beta')

axs[1,0].plot(p_sweep_dps, M_yaw_p_sweep)
axs[1,0].grid(True)
axs[1,0].set_xlabel('Roll Rate [deg/s]')
axs[1,0].set_ylabel(r'$M_z$ [N*m]')
# axs[1].set_title('Yaw Moment vs. Roll Rate')

axs[1,1].plot(r_sweep_dps, M_yaw_r_sweep)
axs[1,1].grid(True)
axs[1,1].set_xlabel('Yaw Rate [deg/s]')
axs[1,1].set_ylabel(r'$M_z$ [N*m]')
# axs[1].set_title('Yaw Moment vs. Yaw Rate')

axs[2,0].plot(delta_a_sweep_deg, M_yaw_delta_a_sweep)
axs[2,0].grid(True)
axs[2,0].set_xlabel('Aileron Deflection [deg]')
axs[2,0].set_ylabel(r'$M_z$ [N*m]')
# axs[2].set_title('Yaw Moment vs. Aileron Deflection')

axs[2,1].plot(delta_r_sweep_deg, M_yaw_delta_r_sweep)
axs[2,1].grid(True)
axs[2,1].set_xlabel('Rudder Deflection [deg]')
axs[2,1].set_ylabel(r'$M_z$ [N*m]')
# axs[2].set_title('Yaw Moment vs. Rudder Deflection')

plt.suptitle(r'Yaw Moment ($n$ or $M_z$) vs. $\beta$, $p$, $r$, $\delta_a$, and $\delta_r$')

plt.show()
