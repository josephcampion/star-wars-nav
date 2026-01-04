
import numpy as np
import matplotlib.pyplot as plt

import models.aerosonde_uav as mav
from models.propulsion import EngineProperties

# TODO: Plot thrust vs. throttle and torque vs. throttle instead.

####################################################
#   Sweep Throttle vs. Thrust and Torque
####################################################

n_throttle_sweep = 50

# Set airspeed and throttle inputs
# TODO: Sweep airspeed range.
Va = 30.0 # [m/s]
throttle_sweep = np.linspace(0.0, 1.0, n_throttle_sweep) # [-]

thrust_prop = np.zeros(n_throttle_sweep)
torque_prop = np.zeros(n_throttle_sweep)

# Make engine model
propeller = EngineProperties([mav.S_prop, \
    mav.k_motor, mav.k_T_p, mav.k_Omega, mav.C_prop])

for i in range(n_throttle_sweep):
    thrust_prop[i] = propeller.get_thrust(Va, throttle_sweep[i])
    torque_prop[i] = propeller.get_gyro_torque(throttle_sweep[i])

_, axs = plt.subplots(2,1)

# TODO: Plot thrust output
axs[0].plot(throttle_sweep, thrust_prop)
axs[0].grid(True)
axs[0].set_ylabel('[N]')
axs[0].set_title('Thrust Output')
axs[0].legend()

# TODO: Plot torque output
axs[1].plot(throttle_sweep, torque_prop)
axs[1].grid(True)
axs[1].set_xlabel('Throttle [-]')
axs[1].set_ylabel('[N*m]')
axs[1].set_title('Torque Output')
axs[1].legend()

####################################################
#   Sweep Airspeed vs. Thrust and Torque
####################################################

n_airspeed_sweep = 50

# Set airspeed and throttle inputs
airspeed_sweep = np.linspace(0.0, 50.0, n_airspeed_sweep) # [m/s]

thrust_prop = np.zeros(n_airspeed_sweep)
torque_prop = np.zeros(n_airspeed_sweep)

# Make engine model
propeller = EngineProperties([mav.S_prop, \
    mav.k_motor, mav.k_T_p, mav.k_Omega, mav.C_prop])

for i in range(n_airspeed_sweep):
    thrust_prop[i] = propeller.get_thrust(airspeed_sweep[i], 1.0)
    torque_prop[i] = propeller.get_gyro_torque(1.0)

_, ax = plt.subplots()

ax.plot(airspeed_sweep, thrust_prop)
ax.grid(True)
ax.set_ylabel('[N]')
ax.set_xlabel('Airspeed [m/s]')
ax.set_title('Thrust Output (Throttle = 1.0)')
ax.legend()

plt.show()
