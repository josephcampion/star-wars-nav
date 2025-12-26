
import numpy as np
from pytest import approx

import models.mass_props as mp
import simulation.kinematics as kin

# M = dH_dt = J*pqr_dot + cross(pqr, J*pqr) 
def solve_f_equals_ma_explicit(x_vec, mass_props, F, M):

    u = x_vec[3]
    v = x_vec[4]
    w = x_vec[5]
    roll = x_vec[6]
    pitch = x_vec[7]
    yaw = x_vec[8]
    p = x_vec[9]
    q = x_vec[10]
    r = x_vec[11]

    m = mass_props.get_mass()
    J = mass_props.get_inertia_matrix()
    Jinv = mass_props.get_inertia_matrix_inv()
    gamma_vec = mass_props.get_gamma_vec()

    Jy = J[1,1]
    gamma = gamma_vec[0]
    gamma1 = gamma_vec[1]
    gamma2 = gamma_vec[2]
    gamma3 = gamma_vec[3]
    gamma4 = gamma_vec[4]
    gamma5 = gamma_vec[5]
    gamma6 = gamma_vec[6]
    gamma7 = gamma_vec[7]
    gamma8 = gamma_vec[8]

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    tp = np.tan(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    pn_dot = cp*cy*u + (sr*sp*cy-cr*sy)*v + (cr*sp*cy+sr*sy)*w
    pe_dot = cp*sy*u + (sr*sp*sy+cr*cy)*v + (cr*sp*sy-sr*cy)*w
    pd_dot = -sp*u + sr*cp*v + cr*cp*w

    udot = F[0] / m - q*w + r*v
    vdot = F[1] / m - r*u + p*w
    wdot = F[2] / m - p*v + q*u

    roll_dot = p + (q*sr + r*cr)*tp
    pitch_dot = q*cr - r*sr
    yaw_dot = (q*sr + r*cr)/cp

    l, m, n = M

    pdot = gamma1 * p * q - gamma2 * q * r + gamma3 * l + gamma4 * n
    qdot = gamma5 * p * r - gamma6 * (p**2 - r**2) + m / Jy
    rdot = gamma7 * p * q - gamma1 * q * r + gamma4 * l + gamma8 * n

    return np.array([
        pn_dot, pe_dot, pd_dot,
        udot, vdot, wdot,
        roll_dot, pitch_dot, yaw_dot,
        pdot, qdot, rdot,
    ])

def test_solve_f_equals_ma():

    mass = 10.0
    Jx = 1.0
    Jy = 2.0
    Jz = 3.0
    Jxz = 4.0
    mass_props = mp.MassProperties(mass, Jx, Jy, Jz, Jxz)

    roll = np.deg2rad(5.0)
    pitch = np.deg2rad(10.0)

    u = 50.0 # [m/s]
    v = 10.0 # [m/s]
    w = -5.0 # [m/s]
    p = -0.3 # [rad/s]
    q = 0.2 # [rad/s]
    r = -0.1 # [rad/s]
    uvw = np.array([u, v, w])
    pqr = np.array([p, q, r])

    F = np.array([6.0, -7.0, 8.0])
    M = np.array([-11.0, 12.0, -13.0])


    x_vec = np.array([0.0, 0.0, 0.0, u, v, w, roll, pitch, 0.0, p, q, r])
    x_ac = kin.NonlinearKinematicState(x_vec)

    x_dot_explicit = solve_f_equals_ma_explicit(x_vec, mass_props, F, M)
    x_dot_implicit = x_ac.solve_f_equals_ma(mass_props, F, M)

    print("\nx_dot_implicit =\n", x_dot_implicit)
    print("x_dot_explicit =\n", x_dot_explicit)

    assert x_dot_implicit[0] == approx(x_dot_explicit[0], abs=1.0e-6)
    assert x_dot_implicit[1] == approx(x_dot_explicit[1], abs=1.0e-6)
    assert x_dot_implicit[2] == approx(x_dot_explicit[2], abs=1.0e-6)
    assert x_dot_implicit[3] == approx(x_dot_explicit[3], abs=1.0e-6)
    assert x_dot_implicit[4] == approx(x_dot_explicit[4], abs=1.0e-6)
    assert x_dot_implicit[5] == approx(x_dot_explicit[5], abs=1.0e-6)
    assert x_dot_implicit[6] == approx(x_dot_explicit[6], abs=1.0e-6)
    assert x_dot_implicit[7] == approx(x_dot_explicit[7], abs=1.0e-6)
    assert x_dot_implicit[8] == approx(x_dot_explicit[8], abs=1.0e-6)
    assert x_dot_implicit[9] == approx(x_dot_explicit[9], abs=1.0e-6)
    assert x_dot_implicit[10] == approx(x_dot_explicit[10], abs=1.0e-6)
    assert x_dot_implicit[11] == approx(x_dot_explicit[11], abs=1.0e-6)

