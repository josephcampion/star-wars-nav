
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
from models.aerodynamics import AerodynamicCoefficients as aero_coeffs
from models.propulsion import EngineProperties as prop
from simulation.dynamics import get_gravity_force

# TODO: Sweep airspeed range and density.
Va = 75.0 # [m/s]
# Not sure what angle of attach would be stall,
# but 15 m/s is too low (needs >20deg aoa).
# TODO: Make this a function of altitude.
rho = 1.2682 # [kg/m^3]

# TODO: Get trim conditions from trim solver or config file.
alpha_trim_deg = -3.5 # [deg]
alpha_trim_rad = np.deg2rad(alpha_trim_deg) # [rad]
q_trim = np.deg2rad(0.0) # [rad/s]
delta_e_trim_deg = -0.0 # [deg]
delta_e_trim_rad = np.deg2rad(delta_e_trim_deg) # [rad]
delta_t_trim = 0.95 # [%]

####################################################
        #   Initialize UAV Parameters
####################################################

m = mav.m

# TODO: Sweep pitch and roll angle.
F_gravity = get_gravity_force(m, 0.0, 0.0)
W = F_gravity[2]
# print("W = ", W)

aero_coeffs = aero_coeffs([
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
#   Sweep angle of attack to get lift and drag forces
####################################################

n_alpha_sweep = 50
alpha_sweep_deg = np.linspace(-10.0, 0.0, n_alpha_sweep)
alpha_sweep_rad = np.deg2rad(alpha_sweep_deg)

F_lift_alpha_sweep = np.zeros(len(alpha_sweep_rad))

for i, alpha in enumerate(alpha_sweep_rad):
    F_lift_alpha_sweep[i] = aero_coeffs.get_lift_force(alpha, q_trim, delta_e_trim_rad, rho, Va, mav.S, mav.c)

# Plot lift force vs. alpha
_, ax_alpha_trim = plt.subplots(1,1)

W_line = W * np.ones(len(alpha_sweep_deg))

alpha_trim_line_y = np.array([np.min(F_lift_alpha_sweep), np.max(F_lift_alpha_sweep)])
alpha_trim_line_x = np.array([alpha_trim_deg, alpha_trim_deg])

ax_alpha_trim.plot(alpha_sweep_deg, F_lift_alpha_sweep, label='Lift')
ax_alpha_trim.plot(alpha_sweep_deg, W_line, 'k--', label='Weight')
ax_alpha_trim.plot(alpha_trim_line_x, alpha_trim_line_y, 'g--', label='Trim Condition')
ax_alpha_trim.grid(True)
ax_alpha_trim.set_xlabel('Alpha [deg]')
ax_alpha_trim.set_ylabel('[N]')
# ax_alpha_trim.set_title('Lift Force vs. Alpha')
ax_alpha_trim.set_title('Lift Force')
ax_alpha_trim.legend()


####################################################
#   Sweep elevator deflection to get pitching moment
####################################################

n_delta_e_sweep = 50
delta_e_sweep_deg = np.linspace(-10.0, 10.0, n_delta_e_sweep)
delta_e_sweep_rad = np.deg2rad(delta_e_sweep_deg)

F_lift_delta_e_sweep = np.zeros(len(delta_e_sweep_rad))

for i, delta_e in enumerate(delta_e_sweep_rad):
    F_lift_delta_e_sweep[i] = aero_coeffs.get_pitch_moment(alpha_trim_rad, q_trim, delta_e, rho, Va, mav.S, mav.c)

delta_e_trim_line_y = np.array([np.min(F_lift_delta_e_sweep), np.max(F_lift_delta_e_sweep)])
delta_e_trim_line_x = np.array([delta_e_trim_deg, delta_e_trim_deg])

_, ax_delta_e_trim = plt.subplots(1,1)

ax_delta_e_trim.plot(delta_e_sweep_deg, F_lift_delta_e_sweep)
ax_delta_e_trim.plot(delta_e_trim_line_x, delta_e_trim_line_y, 'g--', label='Trim Condition')
ax_delta_e_trim.grid(True)
ax_delta_e_trim.set_xlabel('Elevator Deflection [deg] (alpha = 0.0 deg)')
ax_delta_e_trim.set_ylabel('[N]')
ax_delta_e_trim.set_title('Pitching Moment vs. Elevator Deflection')


####################################################
# Throttle vs. Thrust, Drag, and Excess Thrust
####################################################

n_throttle_sweep = 50

# Set airspeed and throttle inputs
# TODO: Sweep airspeed range.
throttle_sweep = np.linspace(0.7, 1.0, n_throttle_sweep) # [-]

F_thrust_prop = np.zeros(n_throttle_sweep)
F_excess_thrust = np.zeros(n_throttle_sweep)

# Make engine model
propeller = prop([mav.S_prop, \
    mav.k_motor, mav.k_T_p, mav.k_Omega, mav.C_prop])

F_drag = aero_coeffs.get_drag_force(alpha_trim_rad, q_trim, delta_e_trim_rad, rho, Va, mav.S, mav.c)
F_drag_line = F_drag * np.ones(len(throttle_sweep))

for i in range(n_throttle_sweep):
    F_thrust_prop[i] = propeller.get_thrust(Va, throttle_sweep[i])
    F_excess_thrust[i] = F_thrust_prop[i] - F_drag

delta_t_trim_line_y = np.array([np.min(F_thrust_prop), np.max(F_thrust_prop)])
delta_t_trim_line_x = np.array([delta_t_trim, delta_t_trim])

_, ax_delta_t_trim = plt.subplots(1,1)

# TODO: Plot thrust output
ax_delta_t_trim.plot(throttle_sweep, F_thrust_prop, label='Thrust')
ax_delta_t_trim.plot(throttle_sweep, F_drag_line, 'k--', label='Drag')
ax_delta_t_trim.plot(throttle_sweep, F_excess_thrust, 'r--', label='Excess Thrust')
ax_delta_t_trim.plot(delta_t_trim_line_x, delta_t_trim_line_y, 'g--', label='Trim Condition')
ax_delta_t_trim.grid(True)
ax_delta_t_trim.set_ylabel('[N]')
ax_delta_t_trim.set_xlabel('Throttle [-]')
ax_delta_t_trim.set_title('Thrust vs. Throttle')
ax_delta_t_trim.legend()

plt.show()
