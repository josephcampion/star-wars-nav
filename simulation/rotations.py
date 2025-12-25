
import numpy as np

class InputAxisError(Exception):
    pass

def rot(angle_rad, axis):

    ca = np.cos(angle_rad)
    sa = np.sin(angle_rad)

    if axis == 0: # x
        return np.array([
            [1.0,   0.0,    0.0],
            [0.0,   ca,     sa],
            [0.0,   -sa,    ca]
        ])
    elif axis == 1: # y
        return np.array([
            [ca,    0.0,    -sa],
            [0.0,   1.0,    0.0],
            [sa,    0.0,    ca]
        ])
    elif axis == 2: # z
        return np.array([
            [ca,    sa,     0.0],
            [-sa,   ca,     0.0],
            [0.0,   0.0,    1.0]
        ])
    else:
        raise InputAxisError("Input axis must be 0, 1, or 2 (for x, y, or z).")

# NOTE: Angles are in radians (since np.sin/cos are too)
def get_dcm_ned2bod(roll, pitch, yaw):
    return rot(roll, 0) @ rot(pitch, 1) @ rot(yaw, 2)

