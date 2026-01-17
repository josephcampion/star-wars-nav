
import numpy as np

###################################################
            #   Initial Conditions
###################################################

# Target and missile velocities
vt0 = 50.0 # [m/s]
vm0 = 100.0 # [m/s]

# Target and missile headings
theta_t0 = np.deg2rad(10.0) # [rad]
theta_m0 = np.deg2rad(20.0) # [rad]

# Range and LOS angle to target
R0 = 1000.0 # [m]
beta0 = np.deg2rad(30.0) # [rad]

# Initial missile position
x_m0 = 0.0 # [m]
y_m0 = 0.0 # [m]

# Initial target position
x_t0 = R0 * np.cos(beta0) # [m]
y_t0 = R0 * np.sin(beta0) # [m]

