
from operator import ge
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
    theta_t_dot = np.deg2rad(2.5) # TODO: Add evasion guidance.

    x_m_dot = params.vm0 * np.cos(theta_m)
    y_m_dot = params.vm0 * np.sin(theta_m)
    theta_m_dot = prop_nav(x)

    return np.array([x_t_dot, y_t_dot, theta_t_dot, x_m_dot, y_m_dot, theta_m_dot])

def get_los_angle(x):
    x_t, y_t, _, x_m, y_m, _ = x
    los_angle = np.arctan2(y_t - y_m, x_t - x_m)
    return los_angle

def get_range(x):
    x_t, y_t, _, x_m, y_m, _ = x
    range = np.hypot(x_t - x_m, y_t - y_m)
    return range

# Assume perfect sensor, for now.
def get_los_rate(x):
    x_t, y_t, theta_t, x_m, y_m, theta_m = x

    R = get_range(x)
    beta = get_los_angle(x)

    vat = params.vt0 * np.sin(beta - theta_t)
    vam = params.vm0 * np.sin(beta - theta_m)   
    beta_dot = -(vat - vam) / R

    return beta_dot

def prop_nav(x):

    beta_dot = get_los_rate(x)

    # TODO: Figure out this gain.

    return 5 * beta_dot

####################################################
#   Initialize Simulation
####################################################

# This can get figured out in Simulation class:
time = np.arange(params.t0, params.Tsim, params.dt)
nt = len(time)

# Initialize vectors 

xdim = [nt, 6]

los_log = np.zeros(nt)
range_log = np.zeros(nt)
los_rate_log = np.zeros(nt)
x_log = np.zeros(xdim)

x_log[0,:] = np.array([
    params.x_t0,
    params.y_t0,
    params.theta_t0,
    params.x_m0,
    params.y_m0,
    params.theta_m0,
])
xdot_log = np.zeros(xdim)

####################################################
#   Propagate Simulation (TODO: Put in separate file/dir)
####################################################

for i in range(nt): 

    x = x_log[i,:]
    xdot = homing_eom(x)

    los_log[i] = get_los_angle(x)
    range_log[i] = get_range(x) 
    los_rate_log[i] = get_los_rate(x)

    xdot_log[i,:] = xdot

    if i < (nt-1):
        x_log[i+1,:] = x_log[i,:] + xdot * params.dt

        # TODO: Stop sim if range is within tolerance
        if range_log[i] < params.Rtol:
            i_end = i
            print("Target reached.")
            break
        else:
            i_end = i


####################################################
#                   Plot Results
####################################################

fig, ax = plt.subplots()

ax.plot(x_log[:i_end,0], x_log[:i_end,1])
ax.set_title("Target Path")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True)

ax.plot(x_log[:i_end,3], x_log[:i_end,4])
ax.set_title("Missile Path")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True)

def plot_r_and_los(ax, x, r, los):

    x_t, y_t, _, x_m, y_m, _ = x
    x_t2 = x_m + r * np.cos(los)
    y_t2 = y_m + r * np.sin(los)

    ax.plot([x_m, x_t2], [y_m, y_t2], 'k:', linewidth=0.25)

for i in range(nt):
    if np.mod(i, 10) == 0:
        plot_r_and_los(ax, x_log[i,:], range_log[i], los_log[i])

fig, ax = plt.subplots(3,1)

ax[0].plot(time[:i_end], range_log[:i_end])
ax[0].set_title("Range")
# ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("Range [m]")
ax[0].grid(True)

ax[1].plot(time[:i_end], np.rad2deg(los_log[:i_end]))
# ax[1].set_title("LOS Angle")
# ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("LOS Angle [deg]")
ax[1].grid(True)

ax[2].plot(time[:i_end], np.rad2deg(los_rate_log[:i_end]), label="Analytical")
# ax[2].set_title("LOS Rate")
ax[2].set_xlabel("Time [s]")
ax[2].set_ylabel("LOS Rate [deg/s]")
ax[2].grid(True)

# Compare against numerical slope
beta_dot_num = (los_log[2:i_end] - los_log[1:(i_end-1)]) / params.dt

ax[2].plot(time[1:(i_end-1)], np.rad2deg(beta_dot_num), label="Numerical")
ax[2].legend()

fig.suptitle("Homing Simulation")

plt.show()
