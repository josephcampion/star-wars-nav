
import numpy as np
# import parameters as IC
from kinematic_state import KinematicState
from plotter import Plotter

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
    #   Input Control (TODO: Combine with sim parameters)
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
    #   Propagate Simulation (TODO: Put in separate file/dir)
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
    
sim_plotter = Plotter(time, X, Xdot, Xddot)
sim_plotter.plot_sim()
