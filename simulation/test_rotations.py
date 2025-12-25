
import numpy as np
from pytest import approx
import simulation.rotations as rt

#---------- Rotation Matrices ------------#

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

