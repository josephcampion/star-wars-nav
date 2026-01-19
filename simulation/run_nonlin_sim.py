
import numpy as np

import models.aerosonde_uav as mav
from models.parameters import initial_condtions as ic
from simulation.kinematics import NonlinearKinematicState as nlks
from simulation.plotter import Plotter
from simulation import dynamics as dyn


####################################################
#   Initialize Vehicle and Simulation
####################################################

uav = mav.vehicle

Tsim = 30.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

# This can get figured out in Simulation class:
time = np.arange(t0, Tsim, dt)
nt = len(time)

# Initialize vectors 
ac_state = nlks(init_conds=ic)

xdim = [nt, len(ac_state.get_state())]

X = np.zeros(xdim)
X[0,:] = ac_state.get_state()
Xdot = np.zeros(xdim)

####################################################
#   Input Forces & Moments (TODO: Add Control)
####################################################

# TODO: Add trim inputs to this (for elevator and throttle).

# amp =  0.1 # [N] or [N*m]
U_control = np.zeros([nt, 4])

# Step Inputs
U_control[:,0] = np.deg2rad(-4.0) * np.ones(nt) # delta_e
U_control[:,1] = 0.4 * np.ones(nt) # delta_t
# U_control[:,2] = amp * np.ones(nt) # delta_a
# U_control[:,3] = amp * np.ones(nt) # delta_r

# Sine Inputs
w_in = 1.0 # [rad/s]
# u_sin_wave = 0.1 * np.sin(w_in * time)
# U_control[:,0] = u_sin_wave # delta_e
# U_control[:,1] = amp * -u_sin_wave # delta_t
# U_control[:,2] = amp * u_sin_wave # delta_a
# U_control[:,3] = amp * u_sin_wave # delta_r

####################################################
#   Propagate Simulation (TODO: Put in separate file/dir)
####################################################

# TODO: Get steady wind and gusts from external file.

for i in range(nt): 

    x = X[i,:]
    ac_state.set_state(x)

    u_control = U_control[i,:]

    F_net, M_net = dyn.get_forces_and_moments(uav, x, u_control)

    xdot = ac_state.solve_f_equals_ma(uav.get_mass_props(), F_net, M_net)

    Xdot[i,:] = xdot

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt
        ac_state.set_state(X[i+1,:])

####################################################
#                   Plot Results
####################################################
    
sim_plotter = Plotter(time, X, Xdot)
sim_plotter.plot_sim()
