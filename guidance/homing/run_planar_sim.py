
import numpy as np
import parameters as params
import matplotlib.pyplot as plt

####################################################
#   Equations of Motion (TODO: Put in separate file/dir)
####################################################

def homing_eom(x): #  u):
    x_t, y_t, theta_t, x_m, y_m, theta_m = x

    x_t_dot = params.vt0 * np.cos(theta_t)
    y_t_dot = params.vt0 * np.sin(theta_t)
    theta_t_dot = 0.0

    x_m_dot = params.vm0 * np.cos(theta_m)
    y_m_dot = params.vm0 * np.sin(theta_m)
    theta_m_dot = 0.0 # TODO: Insert homing guidance law here.

    return np.array([x_t_dot, y_t_dot, theta_t_dot, x_m_dot, y_m_dot, theta_m_dot])

def get_los_angle(x):
    x_t, y_t, _, x_m, y_m, _ = x
    los_angle = np.arctan2(y_t - y_m, x_t - x_m)
    return los_angle

def get_range(x):
    x_t, y_t, _, x_m, y_m, _ = x
    range = np.hypot(x_t - x_m, y_t - y_m)
    return range

####################################################
#   Initialize Simulation
####################################################

# Initialize sim parameters
Tsim = 30.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

# This can get figured out in Simulation class:
time = np.arange(t0, Tsim, dt)
nt = len(time)

# Initialize vectors 

xdim = [nt, 6]
los_log = np.zeros(nt)
range_log = np.zeros(nt)
X = np.zeros(xdim)
X[0,:] = np.array([
    params.x_t0,
    params.y_t0,
    params.theta_t0,
    params.x_m0,
    params.y_m0,
    params.theta_m0,
])
Xdot = np.zeros(xdim)

####################################################
#   Propagate Simulation (TODO: Put in separate file/dir)
####################################################

for i in range(nt): 

    x = X[i,:]
    xdot = homing_eom(x)

    los_log[i] = get_los_angle(x)
    range_log[i] = get_range(x)

    Xdot[i,:] = xdot

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt

####################################################
#                   Plot Results
####################################################

fig, ax = plt.subplots()

ax.plot(X[:,0], X[:,1])
ax.set_title("Target Path")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True)

ax.plot(X[:,3], X[:,4])
ax.set_title("Missile Path")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True)

fig.suptitle("Homing Simulation")

fig, ax = plt.subplots(1,2)
ax[0].plot(time, np.rad2deg(los_log))
ax[0].set_title("LOS Angle")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("LOS Angle [deg]")
ax[0].grid(True)

ax[1].plot(time, range_log)
ax[1].set_title("Range")
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Range [m]")
ax[1].grid(True)

plt.show()
