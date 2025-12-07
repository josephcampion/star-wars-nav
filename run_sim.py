
import numpy as np
import matplotlib.pyplot as plt
# import parameters as IC
from kinematic_state import KinematicState

####################################################
            #   Propagate Simulation
####################################################

# Initialize sim parameters
Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Initialize vectors
initial_state = KinematicState()

xdim = [nt, len(initial_state.get_state())]

X = np.zeros(xdim)
Xdot = np.zeros(xdim)
Xddot = np.zeros(xdim)

####################################################
            #   Input Control 
####################################################

# Input force
amp =  3.0
# w_in = 1.0 # [rad/s]
# U = amp * np.sin(w_in * time)
U = amp * np.ones(xdim)

# natural freq.
wn = 2.0 # [rad/s]

# damping ratio
zeta = 0.3 # []

####################################################
            #   Propagate Simulation
####################################################

for i in range(nt):

    x = X[i,:]
    xdot = Xdot[i,:]
    u = U[i,:]

    # xddot + 2*wn*zeta*xdot + wn**2*x = u
    xddot = u - 2 * wn * zeta * xdot - wn**2 * x
    Xddot[i,:] = xddot

    if i < (nt-1):
        Xdot[i+1,:] = Xdot[i,:] + xddot * dt
        X[i+1,:] = X[i,:] + xdot * dt

####################################################
    #   Plot Results (put this in solo file/dir)
####################################################
    
fig, axs = plt.subplots(3,1)

# ax.axhline(y=0, color="black", linestyle="--")
# ax.axhline(y=0.5, color="black", linestyle=":")
# ax.axhline(y=1.0, color="black", linestyle="--")
# ax.axvline(color="grey")
# ax.axline((0, 0.5), slope=0.25, color="black", linestyle=(0, (5, 5)))

axs[0].plot(time, X[:,1]) # , linewidth=2) # , label=r"$\sigma(t) = \frac{1}{1 + e^{-t}}$")
axs[1].plot(time, Xdot[:,1]) # , linewidth=2)
axs[2].plot(time, Xddot[:,1]) # , linewidth=2)

axs[0].set_title("Position [m]")
axs[1].set_title("Velocity [m/s]")
axs[2].set_title("Acceleration [ms/^2]")

# axs[0].title("Position [m]")
# axs[1].title("Velocity [m]")
axs[2].set_xlabel("Time [s]")

# axs[0].set(xlim=(-10, 10), xlabel="t")
# axs[0].legend(fontsize=14)

axs[0].grid(True)
axs[1].grid(True)
axs[2].grid(True)

plt.show()
