
import numpy as np
from pytest import approx
import models.aerosonde_uav as mav
import models.aerodynamics as aero
from models.propulsion import EngineProperties as prop
from models.mass_props import MassProperties as mass
from models.vehicle import Vehicle
from simulation.dynamics import get_gravity_force, get_forces_and_moments

import models.mass_props as mp
import simulation.kinematics as kin

def get_gravity_force_explicit(m, roll, pitch):

    g = 9.81 # [m/s^2]

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)

    return np.array([
        -m * g * sp,
        m * g * cp * sr,
        m * g * cp * cr
    ])

def test_get_gravity_force():

    mass = 10.0

    # roll = np.deg2rad(5.0)
    # pitch = np.deg2rad(10.0)
    roll = np.random.uniform(low=-np.pi, high=np.pi)
    pitch = np.random.uniform(low=-np.pi/2.0, high=np.pi/2.0)

    F_grav_explicit = get_gravity_force_explicit(mass, roll, pitch)
    F_grav_implicit = get_gravity_force(mass, roll, pitch)

    print("\nF_grav_explicit =\n", F_grav_explicit)
    print("F_grav_implicit =\n", F_grav_implicit)

    assert F_grav_implicit[0] == approx(F_grav_explicit[0], abs=1.0e-6)
    assert F_grav_implicit[1] == approx(F_grav_explicit[1], abs=1.0e-6)
    assert F_grav_implicit[2] == approx(F_grav_explicit[2], abs=1.0e-6)

def test_get_forces_and_moments():

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

    pn = 0.0
    pe = 0.0
    pd = 0.0
    u = 40.0
    v = 0.0
    w = 0.0
    phi = np.deg2rad(0.0)
    theta = np.deg2rad(3.0)
    psi = np.deg2rad(45.0)
    p = np.deg2rad(0.0)
    q = np.deg2rad(0.0)
    r = np.deg2rad(0.0)
    state = np.array([pn, pe, pd, u, v, w, phi, theta, psi, p, q, r])

    delta_e = 0.1
    delta_t = 1.0
    delta_a = 0.0
    delta_r = 0.0
    control = np.array([delta_e, delta_t, delta_a, delta_r])

    F_total, M_total = get_forces_and_moments(vehicle, state, control)

    print("F_total = ", F_total)
    print("M_total = ", M_total)

