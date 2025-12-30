
# This feels like a lot of imports...is there a way to consolidate?
import numpy as np
import models.aerosonde_uav as mav
import models.aerodynamics as aero
from models.propulsion import EngineProperties as prop
from models.mass_props import MassProperties as mass
from models.parameters import initial_condtions as ic
from models.vehicle import Vehicle
from simulation.kinematics import NonlinearKinematicState 
from plotter import Plotter
from simulation import dynamics as dyn

####################################################
            #   Initialize Vehicle
####################################################

aero_coeffs = aero.AerodynamicCoefficients([
    mav.C_L_0, mav.C_D_0, mav.C_m_0, \
    mav.C_L_alpha, mav.C_D_alpha, mav.C_m_alpha, \
    mav.C_L_q, mav.C_D_q, mav.C_m_q, \
    mav.C_L_delta_e, mav.C_D_delta_e, mav.C_m_delta_e, \
    mav.C_Y_0, mav.C_l_0, mav.C_n_0, \
    mav.C_Y_beta, mav.C_l_beta, mav.C_n_beta, \
    mav.C_Y_p, mav.C_l_p, mav.C_n_p, \
    mav.C_Y_r, mav.C_l_r, mav.C_n_r, \
    mav.C_Y_delta_a, mav.C_l_delta_a, mav.C_n_delta_a, \
    mav.C_Y_delta_r, mav.C_l_delta_r, mav.C_n_delta_r \
])

engine_props = prop([mav.S_prop, \
    mav.k_motor, mav.k_T_p, mav.k_Omega, mav.C_prop])

mass_props = mass(mav.m, mav.Jx, mav.Jy, mav.Jz, mav.Jxz)

vehicle = Vehicle(aero_coeffs, engine_props, mass_props, mav.S, mav.c, mav.b, mav.rho)


####################################################
#   Initialize Simulation
####################################################

# Initialize sim parameters
Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

# This can get figured out in Simulation class:
time = np.arange(t0, Tsim, dt)
nt = len(time)

mass_props = mass(mav.m, mav.Jx, mav.Jy, mav.Jz, mav.Jxz)

# Initialize vectors 
ac_state = NonlinearKinematicState(init_conds=ic)

xdim = [nt, len(ac_state.get_state())]

X = np.zeros(xdim)
X[0,:] = ac_state.get_state()
Xdot = np.zeros(xdim)

####################################################
#   Input Forces & Moments (TODO: Add Control)
####################################################

# TODO: Add trim inputs to this (for elevator and throttle).

amp =  0.5 # [N] or [N*m]
U_control = np.zeros([nt, 4])

# Step Inputs
U_control[:,0] = amp * np.ones(nt) # delta_e
U_control[:,1] = 1.0 * amp * np.ones(nt) # delta_t
# U_control[:,2] = amp * np.ones(nt) # delta_a
# U_control[:,3] = amp * np.ones(nt) # delta_r

# Sine Inputs
# w_in = 1.0 # [rad/s]
# u_sin_wave = amp * np.sin(w_in * time)
# U_control[:,0] = amp * u_sin_wave # delta_e
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

    F_net, M_net = dyn.get_forces_and_moments(vehicle, x, u_control)
    # print("F_net = ", F_net)
    # print("M_net = ", M_net)
    xdot = ac_state.solve_f_equals_ma(mass_props, F_net, M_net)

    Xdot[i,:] = xdot

    if i < (nt-1):
        X[i+1,:] = X[i,:] + xdot * dt
        ac_state.set_state(X[i+1,:])

####################################################
#                   Plot Results
####################################################
    
sim_plotter = Plotter(time, X, Xdot)
sim_plotter.plot_sim()
