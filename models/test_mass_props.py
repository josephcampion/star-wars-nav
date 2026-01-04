
import numpy as np
from pytest import approx

import models.mass_props as mp

def test_mass_props():
    mass = 10.0
    Jx = 1.0
    Jy = 2.0
    Jz = 3.0
    Jxz = 4.0
    mass_props = mp.MassProperties(mass, Jx, Jy, Jz, Jxz)

    assert mass_props.get_mass() == mass

    J = mass_props.get_inertia_matrix()
    assert J[0,0] == Jx
    assert J[1,1] == Jy
    assert J[2,2] == Jz
    assert J[0,2] == -Jxz
    assert J[2,0] == -Jxz

    J_inv_numerical = np.linalg.inv(J)

    J_inv = mass_props.get_inertia_matrix_inv()
    assert J_inv[0,0] == J_inv_numerical[0,0]
    assert J_inv[0,1] == J_inv_numerical[0,1]
    assert J_inv[1,0] == J_inv_numerical[1,0]
    assert J_inv[1,1] == J_inv_numerical[1,1]
    assert J_inv[2,0] == J_inv_numerical[2,0]
    assert J_inv[2,1] == J_inv_numerical[2,1]
    assert J_inv[2,2] == J_inv_numerical[2,2]

# Assuming M = [l, m, n]^T is zero.
def get_pqr_dot_explicit(gamma_vec, p, q, r):

    # gamma = gamma_vec[0]
    gamma1 = gamma_vec[1]
    gamma2 = gamma_vec[2]
    # gamma3 = gamma_vec[3]
    # gamma4 = gamma_vec[4]
    gamma5 = gamma_vec[5]
    gamma6 = gamma_vec[6]
    gamma7 = gamma_vec[7]
    # gamma8 = gamma_vec[8]

    pdot = gamma1 * p * q - gamma2 * q * r
    qdot = gamma5 * p * r - gamma6 * (p**2 - r**2)
    rdot = gamma7 * p * q - gamma1 * q * r

    return np.array([pdot, qdot, rdot])

def test_gamma_vec():
    mass = 10.0
    Jx = 1.0
    Jy = 2.0
    Jz = 3.0
    Jxz = 4.0
    mass_props = mp.MassProperties(mass, Jx, Jy, Jz, Jxz)

    p = 0.1 # [rad/s]
    q = 0.2 # [rad/s]
    r = 0.3 # [rad/s]
    pqr = np.array([p, q, r])

    gamma_vec = mass_props.get_gamma_vec()
    pqr_dot_explicit = get_pqr_dot_explicit(gamma_vec, p, q, r)

    J = mass_props.get_inertia_matrix()
    Jinv = mass_props.get_inertia_matrix_inv()

    # J*pqr_dot + cross(pqr, J*pqr) = M (M = 0 in this case)
    # J*pqr_dot = -cross(pqr, J*pqr)
    pqr_dot_implicit = Jinv @ (-np.cross(pqr, J @ pqr))

    print("\npqr_dot_explicit = ", pqr_dot_explicit)
    print("pqr_dot_implicit = ", pqr_dot_implicit)

    assert pqr_dot_explicit[0] == approx(pqr_dot_implicit[0], abs=1.0e-6)
    assert pqr_dot_explicit[1] == approx(pqr_dot_implicit[1], abs=1.0e-6)
    assert pqr_dot_explicit[2] == approx(pqr_dot_implicit[2], abs=1.0e-6)
