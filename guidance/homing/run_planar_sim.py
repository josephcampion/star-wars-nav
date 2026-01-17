
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

X = np.zeros(xdim)
X[0,:] = np.array([params.x_t0, params.y_t0, params.theta_t0, params.x_m0, params.y_m0, params.theta_m0])
Xdot = np.zeros(xdim)

####################################################
#   Propagate Simulation (TODO: Put in separate file/dir)
####################################################

for i in range(nt): 

    x = X[i,:]
    xdot = homing_eom(x)

    Xdot[i,:] = xdot

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt

####################################################
#                   Plot Results
####################################################

fig, ax = plt.subplots(2,3)

ax[0,0].plot(time, X[:,0])
ax[0,1].plot(time, X[:,1])
ax[0,2].plot(time, X[:,2])
ax[1,0].plot(time, X[:,3])
ax[1,1].plot(time, X[:,4])
ax[1,2].plot(time, X[:,5])

ax[0,0].set_title("Target X-Position")
ax[0,1].set_title("Target Y-Position")
ax[0,2].set_title("Target Heading")
ax[1,0].set_title("Missile X-Position")
ax[1,1].set_title("Missile Y-Position")
ax[1,2].set_title("Missile Heading")

for i in range(2):
    for j in range(3):
        ax[i,j].grid(True)

ax[0,0].set_xlabel("Time [s]")
ax[0,1].set_xlabel("Time [s]")
ax[0,2].set_xlabel("Time [s]")
ax[1,0].set_xlabel("Time [s]")
ax[1,1].set_xlabel("Time [s]")
ax[1,2].set_xlabel("Time [s]")

fig.suptitle("Homing Simulation")

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

plt.show()
