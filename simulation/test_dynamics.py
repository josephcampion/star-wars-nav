
import numpy as np
from pytest import approx
from simulation.dynamics import get_gravity_force

import models.mass_props as mp
import simulation.kinematics as kin

def get_gravity_force_explicit(m, roll, pitch):

    g = 9.81 # [m/s^2]

    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)

    return np.array([-m * g * sp, m * g * cp * sr, m * g * cp * cr])


def test_solve_f_equals_ma():

    mass = 10.0

    roll = np.deg2rad(5.0)
    pitch = np.deg2rad(10.0)

    F_grav_explicit = get_gravity_force_explicit(mass, roll, pitch)
    F_grav_implicit = get_gravity_force(mass, roll, pitch)

    print("\nF_grav_explicit =\n", F_grav_explicit)
    print("F_grav_implicit =\n", F_grav_implicit)

    assert F_grav_implicit[0] == approx(F_grav_explicit[0], abs=1.0e-6)
    assert F_grav_implicit[1] == approx(F_grav_explicit[1], abs=1.0e-6)
    assert F_grav_implicit[2] == approx(F_grav_explicit[2], abs=1.0e-6)


