
import numpy as np
import models.parameters as parameters
from simulation.kinematics import LinearKinematicState 
from plotter import Plotter
from simulation import dynamics

####################################################
#   Initialize Simulation (TODO: move this to 'simulation')
####################################################

# TODO: Turn this into sim = Simulation(dt,T,dyn_fun)

# Initialize sim parameters
Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

# This can get figured out in Simulation class:
time = np.arange(t0, Tsim, dt)
nt = len(time)

# Initialize vectors
ac_state = LinearKinematicState(init_conds=parameters.initial_condtions)

xdim = [nt, len(ac_state.get_state())]

X = np.zeros(xdim)
X[0,:] = ac_state.get_state()
Xdot = np.zeros(xdim)

lon_dyn = dynamics.LinearDynamics4x4(
    parameters.A_lon,
    parameters.B_lon,
    np.eye(4),
    np.zeros([4,2]))

lat_dyn = dynamics.LinearDynamics4x4(
    parameters.A_lat,
    parameters.B_lat,
    np.eye(4),
    np.zeros([4,2]))

####################################################
#   Input Control (TODO: Combine with sim parameters)
####################################################

# Input force
amp =  1.0
# w_in = 1.0 # [rad/s]
# U = amp * np.sin(w_in * time)
U = amp * np.ones(xdim)

####################################################
#   Propagate Simulation (TODO: Put in separate file/dir)
####################################################

# TODO: turn this into 'sim.run()'

for i in range(nt): 

    x = X[i,:]
    ac_state.set_state(x)
    x_lon = ac_state.get_lon_state()
    x_lat = ac_state.get_lat_state()

    # u = np.array([1.0, 1.0]) # step input
    u = np.array([1.0, 0.0]) # step input

    xdot_lon = lon_dyn.propagate_state(x_lon, u)
    xdot_lat = lat_dyn.propagate_state(x_lat, u)

    xdot = np.array([
        0.0, # TODO: insert eqn. for pn_dot
        0.0, # TODO: insert eqn. for pe_dot
        0.0, # TODO: insert eqn. for pd_dot
        xdot_lon[0],
        xdot_lat[0],
        xdot_lon[1],
        xdot_lat[2],
        xdot_lon[3], # really q, or x_lon[2]
        0.0, # really r, or x_lat[3]
        xdot_lat[1],
        xdot_lon[2],
        xdot_lat[3],
    ])

    Xdot[i,:] = xdot

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt
        ac_state.set_state(X[i+1,:])

####################################################
#   Plot Results (put this in solo file/dir)
####################################################
    
sim_plotter = Plotter(time, X, Xdot)
sim_plotter.plot_sim()
