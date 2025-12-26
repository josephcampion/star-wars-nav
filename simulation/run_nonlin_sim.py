
import numpy as np
import models.aerosonde_uav as mav
from models.parameters import initial_condtions as ic
from models.mass_props import MassProperties
from simulation.kinematics import NonlinearKinematicState 
from plotter import Plotter
from simulation import dynamics

####################################################
#   Initialize Simulation (TODO: move this to 'simulation')
####################################################

# Initialize sim parameters
Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

# This can get figured out in Simulation class:
time = np.arange(t0, Tsim, dt)
nt = len(time)

mass_props = MassProperties(mav.m, mav.Jx, mav.Jy, mav.Jz, mav.Jxz)

# Initialize vectors
ac_state = NonlinearKinematicState(init_conds=ic)

xdim = [nt, len(ac_state.get_state())]

X = np.zeros(xdim)
X[0,:] = ac_state.get_state()
Xdot = np.zeros(xdim)

####################################################
#   Input Forces & Moments (TODO: Control)
####################################################

amp =  1.0 # [N] or [N*m]
w_in = 1.0 # [rad/s]
u_sin_wave = np.sin(w_in * time)

U = np.zeros([nt, 6])
U[:,0] = amp * u_sin_wave # Fx
U[:,1] = amp * -u_sin_wave # Fy
U[:,2] = amp * u_sin_wave # Fz
U[:,3] = amp * -u_sin_wave # Mx
U[:,4] = amp * u_sin_wave # My
U[:,5] = amp * -u_sin_wave # Mz

####################################################
#   Propagate Simulation (TODO: Put in separate file/dir)
####################################################

for i in range(nt): 

    x = X[i,:]
    ac_state.set_state(x)

    u = U[i,:]

    xdot = ac_state.solve_f_equals_ma(mass_props, u[0:3], u[3:6])

    Xdot[i,:] = xdot

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt
        ac_state.set_state(X[i+1,:])

####################################################
#   Plot Results (put this in solo file/dir)
####################################################
    
sim_plotter = Plotter(time, X, Xdot)
sim_plotter.plot_sim()
