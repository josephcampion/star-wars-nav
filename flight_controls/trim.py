
import numpy as np


# Map of airspeed to elevator deflection [m/s -> rad]

airspeed_trim_indeps = np.array([
    20.0,
    25.0,
    30.0,
    35.0,
    40.0,
    45.0,
    50.0,
    55.0,
    60.0,
    75.0,
])

# TODO: Make this a function of altitude.
air_density_trim_indeps = 1.2682 # [kg/m^3]

# TODO: Update this for nonlinear C_L vs. alpha.
alpha_trim_deps = np.array([
    10.0,
    5.0,
    2.5,
    0.0,
    -1.0,
    -2.0,
    -2.0,
    -3.0,
    -3.0,
    -3.5,
])

elevator_trim_deps = np.array([
    np.deg2rad(-10.0),
    np.deg2rad(-6.5),
    np.deg2rad(-4.0),
    np.deg2rad(-2.5),
    np.deg2rad(-2.0),
    np.deg2rad(-1.0),
    np.deg2rad(-1.0),
    np.deg2rad(-0.5),
    np.deg2rad(-0.5),
    np.deg2rad(0.0),
])

throttle_trim_deps = np.array([
    0.28,
    0.34,
    0.40,
    0.46,
    0.53,
    0.58,
    0.64,
    0.70,
    0.76,
    0.95,
])

trim_map = np.stack([
    airspeed_trim_indeps,
    alpha_trim_deps,
    elevator_trim_deps,
    throttle_trim_deps
    ], axis=1)

print("trim_map = ", trim_map)