
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
from models.propulsion import EngineProperties

# TODO: Plot thrust vs. throttle and torque vs. throttle instead.

####################################################
    #   Initialize Simulation Parameters
####################################################

Tsim = 60.0 # [sec]
dt = 0.1 # timestep [sec]
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Set airspeed and throttle inputs
# TODO: Sweep airspeed range.
Va = 20.0 # [m/s]
delta_t = np.linspace(0.0, 1.0, nt) # [-]

thrust_prop = np.zeros(nt)
torque_prop = np.zeros(nt)

####################################################
    #   Run Simulation of Engine Model
####################################################

# Make engine model
propeller = EngineProperties([mav.S_prop, \
    mav.k_motor, mav.k_T_p, mav.k_Omega, mav.C_prop])

for i in range(nt):
    thrust_prop[i] = propeller.get_thrust(Va, delta_t[i])
    torque_prop[i] = propeller.get_gyro_torque(delta_t[i])

####################################################
            #   Plot Results
####################################################

_, axs = plt.subplots(3,1)

# TODO: Plot throttle input
axs[0].plot(time, 100.0 * delta_t)
axs[0].grid(True)
# axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('[%]')
axs[0].set_title('Throttle Input (%)')
axs[0].legend()

# TODO: Plot thrust output
axs[1].plot(time, thrust_prop)
axs[1].grid(True)
# axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('[N]')
axs[1].set_title('Thrust Output')
axs[1].legend()

# TODO: Plot torque output
axs[2].plot(time, torque_prop)
axs[2].grid(True)
# axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('[N*m]')
axs[2].set_title('Torque Output')
axs[2].legend()

plt.show()
