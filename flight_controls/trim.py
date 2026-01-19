
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
    np.deg2rad(10.0),
    np.deg2rad(5.0),
    np.deg2rad(2.5),
    np.deg2rad(0.0),
    np.deg2rad(-1.0),
    np.deg2rad(-2.0),
    np.deg2rad(-2.0),
    np.deg2rad(-3.0),
    np.deg2rad(-3.0),
    np.deg2rad(-3.5),
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

# TODO: Make this more flexible for turns, climbs, etc.
def get_trim_conditions(Va):
    trim_ind = np.argmin(np.abs(airspeed_trim_indeps - Va))
    alpha_trim = alpha_trim_deps[trim_ind]
    delta_e_trim = elevator_trim_deps[trim_ind]
    delta_t_trim = throttle_trim_deps[trim_ind]

    beta_trim = 0.0 # [rad]

    u_trim = Va * np.cos(alpha_trim)
    w_trim = Va * np.sin(alpha_trim)

    v_trim = 0.0 # [m/s]
    p_trim = 0.0 # [rad/s]
    q_trim = 0.0 # [rad/s]
    r_trim = 0.0 # [rad/s]

    delta_a_trim = 0.0 # [rad] 
    delta_r_trim = 0.0 # [rad]
    theta_trim = alpha_trim # assume steady level flight for now

    return np.array([
        alpha_trim,
        beta_trim,
        u_trim,
        v_trim,
        w_trim,
        theta_trim,
        p_trim,
        q_trim,
        r_trim,
        delta_e_trim,
        delta_t_trim,
        delta_a_trim,
        delta_r_trim
    ])

    """ From MAV textbook, Appendix F3.
    # inputs: alpha_trim, beta_trim, phi_trim, Va_trim, R_trim gamma_trim, 
    u_trim = Va * np.cos(alpha_trim) * np.cos(beta_trim)
    v_trim = Va * np.sin(beta_trim)
    w_trim = Va * np.sin(alpha_trim) * np.cos(beta_trim)
    theta_trim = alpha_trim + gamma_trim
    p_trim = -Va_trim / R_trim * np.sin(theta_trim)
    q_trim = Va_trim / R_trim  * np.sin(phi_trim) * np.cos(theta_trim)
    r_trim = Va_trim / R_trim * np.cos(phi_trim) * np.cos(theta_trim)
    # delta_a_trim = ???
    # delta_r_trim = ???
    """

if __name__ == "__main__":
    print("airspeed_trim_indeps = ", airspeed_trim_indeps)
    print("alpha_trim_deps = ", alpha_trim_deps)
    print("elevator_trim_deps = ", elevator_trim_deps)
    print("throttle_trim_deps = ", throttle_trim_deps)
    print("trim_map = ", trim_map)
