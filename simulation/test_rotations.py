
import numpy as np
from pytest import approx
import simulation.rotations as rt


####################################################
        #   NED to Body Frame Rotation
####################################################

def get_dcm_ned2bod_explicit(roll, pitch, yaw):

    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    return np.array([
        [cp*cy, cp*sy, -sp],
        [sr*sp*cy-cr*sy, sr*sp*sy+cr*cy, sr*cp],
        [cr*sp*cy+sr*sy, cr*sp*sy-sr*cy, cr*cp]
    ])

# NOTE: Run 'pytest -s' to see print statements.
def test_get_dcm_ned2bod():

    roll = np.random.uniform(low=-np.pi, high=np.pi)
    pitch = np.random.uniform(low=-np.pi/2.0, high=np.pi/2.0)
    yaw = np.random.uniform(low=-2.0*np.pi, high=2.0*np.pi)

    print("\nroll = ", np.rad2deg(roll), " [deg]")
    print("pitch = ", np.rad2deg(pitch), " [deg]")
    print("yaw = ", np.rad2deg(yaw), " [deg]\n")

    DCM_ned2bod = rt.get_dcm_ned2bod(roll, pitch, yaw)

    DCM_ned2bod_explicit = get_dcm_ned2bod_explicit(roll, pitch, yaw)

    print("DCM_ned2bod =\n", DCM_ned2bod)
    print("DCM_ned2bod_explicit =\n", DCM_ned2bod_explicit)

    assert DCM_ned2bod[0,0] == approx(DCM_ned2bod_explicit[0,0], abs=1.0e-6)
    assert DCM_ned2bod[0,1] == approx(DCM_ned2bod_explicit[0,1], abs=1.0e-6)
    assert DCM_ned2bod[0,2] == approx(DCM_ned2bod_explicit[0,2], abs=1.0e-6)
    assert DCM_ned2bod[1,0] == approx(DCM_ned2bod_explicit[1,0], abs=1.0e-6)
    assert DCM_ned2bod[1,1] == approx(DCM_ned2bod_explicit[1,1], abs=1.0e-6)
    assert DCM_ned2bod[1,2] == approx(DCM_ned2bod_explicit[1,2], abs=1.0e-6)
    assert DCM_ned2bod[2,0] == approx(DCM_ned2bod_explicit[2,0], abs=1.0e-6)
    assert DCM_ned2bod[2,1] == approx(DCM_ned2bod_explicit[2,1], abs=1.0e-6)
    assert DCM_ned2bod[2,2] == approx(DCM_ned2bod_explicit[2,2], abs=1.0e-6)


####################################################
#   Body Angular Rates (p, q, r) to RPY Derivatives
####################################################

# Using this inverse relationship because it's easier to derive.
def get_rpy_dot_pdr(roll, pitch):

    DCM_roll = rt.rot(roll, 0)
    DCM_pitch = rt.rot(pitch, 1)

    col1 = np.array([1.0, 0.0, 0.0])
    col2 = DCM_roll @ np.array([0.0, 1.0, 0.0])
    col3 = DCM_roll @ DCM_pitch @ np.array([0.0, 0.0, 1.0])

    print("col1 = ", col1)
    print("col2 = ", col2)
    print("col3 = ", col3)

    A_rpy_dot_to_pqr = np.column_stack((col1, col2, col3))

    print("A_rpy_dot_to_pqr =\n", A_rpy_dot_to_pqr)

    return A_rpy_dot_to_pqr

def test_get_pqr_to_rpy_dot():

    roll = np.random.uniform(low=-np.pi, high=np.pi)
    pitch = np.random.uniform(low=-np.pi/2.0, high=np.pi/2.0)
    # yaw = np.random.uniform(low=-2.0*np.pi, high=2.0*np.pi)

    A_rpy_dot_to_pqr = get_rpy_dot_pdr(roll, pitch)

    # NOTE: In practice, numerical inv not recommended
    # Solve A@x=b instead.
    A_pqr_to_rpy_dot_numerical = np.linalg.inv(A_rpy_dot_to_pqr)

    A_pqr_to_rpy_dot = rt.get_pqr_to_rpy_dot(roll, pitch)

    I_check = A_rpy_dot_to_pqr @ A_pqr_to_rpy_dot

    print("A_pqr_to_rpy_dot_numerical =\n", A_pqr_to_rpy_dot_numerical)
    print("(rt) A_pqr_to_rpy_dot =\n", A_pqr_to_rpy_dot)
    print("(rt) I_check =\n", I_check)

    assert I_check[0,0] == approx(1.0, abs=1.0e-6)
    assert I_check[0,1] == approx(0.0, abs=1.0e-6)
    assert I_check[0,2] == approx(0.0, abs=1.0e-6)
    assert I_check[1,0] == approx(0.0, abs=1.0e-6)
    assert I_check[1,1] == approx(1.0, abs=1.0e-6)
    assert I_check[1,2] == approx(0.0, abs=1.0e-6)
    assert I_check[2,0] == approx(0.0, abs=1.0e-6)
    assert I_check[2,1] == approx(0.0, abs=1.0e-6)
    assert I_check[2,2] == approx(1.0, abs=1.0e-6)

    assert A_pqr_to_rpy_dot[0,0] == approx(A_pqr_to_rpy_dot_numerical[0,0], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[0,1] == approx(A_pqr_to_rpy_dot_numerical[0,1], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[0,2] == approx(A_pqr_to_rpy_dot_numerical[0,2], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[1,0] == approx(A_pqr_to_rpy_dot_numerical[1,0], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[1,1] == approx(A_pqr_to_rpy_dot_numerical[1,1], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[1,2] == approx(A_pqr_to_rpy_dot_numerical[1,2], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[2,0] == approx(A_pqr_to_rpy_dot_numerical[2,0], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[2,1] == approx(A_pqr_to_rpy_dot_numerical[2,1], abs=1.0e-6)
    assert A_pqr_to_rpy_dot[2,2] == approx(A_pqr_to_rpy_dot_numerical[2,2], abs=1.0e-6)

