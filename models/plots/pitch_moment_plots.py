
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
#   Sweep angle of attack for get pitching moment.
####################################################

q = np.deg2rad(0.0) # [rad/s]
delta_e = 0.0 # [rad]

n_alpha_sweep = 50
alpha_sweep_deg = np.linspace(-10.0, 20.0, n_alpha_sweep)
alpha_sweep_rad = np.deg2rad(alpha_sweep_deg)

M_pitch_alpha_sweep = np.zeros(len(alpha_sweep_rad))

for i, alpha in enumerate(alpha_sweep_rad):
    M_pitch_alpha_sweep[i] = aero_coeffs.get_pitch_moment(alpha, q, delta_e, rho, Va, mav.S, mav.c)

####################################################
#   Sweep pitch rate to get pitching moment.
####################################################

alpha = np.deg2rad(2.0) # [rad]
delta_e = np.deg2rad(0.0) # [rad]

n_q_sweep = 50
q_sweep_dps = np.linspace(-10.0, 10.0, n_q_sweep) # [deg/s]
q_sweep_rps = np.deg2rad(q_sweep_dps) # [rad/s]

M_pitch_q_sweep = np.zeros(len(q_sweep_rps))

for i, q in enumerate(q_sweep_rps):
    M_pitch_q_sweep[i] = aero_coeffs.get_pitch_moment(alpha, q, delta_e, rho, Va, mav.S, mav.c)

####################################################
#   Sweep elevator deflection to get lift and drag forces
####################################################

alpha = np.deg2rad(2.0) # [rad]
q = np.deg2rad(0.0) # [rad/s]

n_delta_e_sweep = 50
delta_e_sweep_deg = np.linspace(-10.0, 10.0, n_delta_e_sweep)
delta_e_sweep_rad = np.deg2rad(delta_e_sweep_deg)

M_pitch_delta_e_sweep = np.zeros(len(delta_e_sweep_rad))

for i, delta_e in enumerate(delta_e_sweep_rad):
    M_pitch_delta_e_sweep[i] = aero_coeffs.get_pitch_moment(alpha, q, delta_e, rho, Va, mav.S, mav.c)

####################################################
            #   Plot Results
####################################################

_, axs = plt.subplots(3,1)

axs[0].plot(alpha_sweep_deg, M_pitch_alpha_sweep)
axs[0].grid(True)
axs[0].set_xlabel('Alpha [deg]')
axs[0].set_ylabel('[N*m]')
# axs[0].set_title('Lift Force vs. Alpha')
axs[0].set_title('Pitching Moment (m or My)')

axs[1].plot(q_sweep_dps, M_pitch_q_sweep)
axs[1].grid(True)
axs[1].set_xlabel('Pitch Rate [deg/s]')
axs[1].set_ylabel('[N*m]')
# axs[1].set_title('Lift Force vs. Pitch Rate')

axs[2].plot(delta_e_sweep_deg, M_pitch_delta_e_sweep)
axs[2].grid(True)
axs[2].set_xlabel('Elevator Deflection [deg]')
axs[2].set_ylabel('[N*m]')
# axs[2].set_title('Lift Force vs. Elevator Deflection')

plt.show()
