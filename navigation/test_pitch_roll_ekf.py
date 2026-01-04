
import numpy as np

from navigation.pitch_roll_ekf import PitchRollEKF, GRAV_ACCEL_MPS2
from models.sensors import get_y_accel_meas

# Initialize EKF
phi0 = np.deg2rad(10.0)
theta0 = np.deg2rad(10.0)
x_hat_pred = np.array([phi0, theta0]) # + random_init 
P_pred = np.eye(2) * 1.e-2
x_hat_upd = x_hat_pred.copy()
P_upd = P_pred.copy()
Q = np.eye(2) * 1.e-4
R = np.eye(3) * 1.e-2
ekf = PitchRollEKF(Q, R)

# TODO: Move this to test_sensors.py
def test_get_y_accel_meas():

    # x = [u, v, w, phi, theta, p, q, r]
    # xdot = [udot, vdot, wdot]

    #--------------- No motion, no pitch or roll. ---------------#
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    xdot = np.array([0.0, 0.0, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)

    assert np.isclose(y_accel_meas[0], 0.0, 1.0e-6)
    assert np.isclose(y_accel_meas[1], 0.0, 1.0e-6)
    assert np.isclose(y_accel_meas[2], -GRAV_ACCEL_MPS2, 1.0e-6)

    #--------------- No motion, pitched nose up. ---------------#
    x = np.array([0.0, 0.0, 0.0, 0.0, np.deg2rad(45.0), 0.0, 0.0, 0.0])
    xdot = np.array([0.0, 0.0, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)

    ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
    assert np.isclose(y_accel_meas[0], ans, 1.0e-6)
    assert np.isclose(y_accel_meas[1], 0.0, 1.0e-6)
    assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

    #--------------- No motion, rolled right. ---------------#
    x = np.array([0.0, 0.0, 0.0, np.deg2rad(45.0), 0.0, 0.0, 0.0, 0.0])
    xdot = np.array([0.0, 0.0, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)

    ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
    assert np.isclose(y_accel_meas[0], 0.0, 1.0e-6)
    assert np.isclose(y_accel_meas[1], -ans, 1.0e-6)
    assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

    #--------------- Full steam ahead (udot>0). ---------------#
    x = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    udot_ans = 10.0
    xdot = np.array([udot_ans, 0.0, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)

    ans = GRAV_ACCEL_MPS2
    assert np.isclose(y_accel_meas[0], udot_ans, 1.0e-6)
    assert np.isclose(y_accel_meas[1], 0.0, 1.0e-6)
    assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

    #--------------- Starboard acceleration (vdot>0). ---------------#
    x = np.array([0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vdot_ans = 10.0
    xdot = np.array([0.0, vdot_ans, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)
    ans = GRAV_ACCEL_MPS2
    assert np.isclose(y_accel_meas[0], 0.0, 1.0e-6)
    assert np.isclose(y_accel_meas[1], vdot_ans, 1.0e-6)
    assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

def test_get_y_accel_est():

    # Compare with get_y_accel_meas (under conditions with no wind or acceleration).
    # Estimator assumes udot=vdot=wdot=0, alpha=theta, beta=0.

    # x = [phi, theta]^T
    # u = [Va, p, q, r]^T

    #--------------- No motion, no pitch or roll. ---------------#
    x = np.array([0.0, 0.0])
    xdot = np.array([0.0, 0.0, 0.0, 0.0])

    y_accel_est = ekf.get_y_accel_est(x, xdot)

    assert np.isclose(y_accel_est[0], 0.0, 1.0e-6)
    assert np.isclose(y_accel_est[1], 0.0, 1.0e-6)
    assert np.isclose(y_accel_est[2], -GRAV_ACCEL_MPS2, 1.0e-6)

    #--------------- No motion, pitched nose up. ---------------#
    x = np.array([0.0, np.deg2rad(45.0)])
    u = np.array([10.0, 0.0, 0.0, 0.0])

    y_accel_est = ekf.get_y_accel_est(x, u)

    ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
    assert np.isclose(y_accel_est[0], ans, 1.0e-6)
    assert np.isclose(y_accel_est[1], 0.0, 1.0e-6)
    assert np.isclose(y_accel_est[2], -ans, 1.0e-6)

    #--------------- No motion, rolled right. ---------------#
    x = np.array([np.deg2rad(45.0), 0.0])
    u = np.array([10.0, 0.0, 0.0, 0.0])

    y_accel_est = ekf.get_y_accel_est(x, u)

    ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
    assert np.isclose(y_accel_est[0], 0.0, 1.0e-6)
    assert np.isclose(y_accel_est[1], -ans, 1.0e-6)
    assert np.isclose(y_accel_est[2], -ans, 1.0e-6)

    #--------------- Random motion. ---------------#
    phi = np.deg2rad(10.0)
    theta = np.deg2rad(20.0)
    p, q, r = 0.1, -0.2, 0.3
    Va = 25.0
    u = np.array([Va, p, q, r])
    x = np.array([phi, theta])
    y_accel_est = ekf.get_y_accel_est(x, u)
    # print("\ny_accel_est =", y_accel_est)

    # Estimator assumes u = Va*cos(theta), v = 0, w = Va*sin(theta)
    u = Va * np.cos(theta)
    v = 0.0
    w = Va * np.sin(theta)
    x = np.array([u, v, w, phi, theta, p, q, r])
    # Estimator assumes udot = 0, vdot = 0, wdot = 0
    xdot = np.array([0.0, 0.0, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)
    # print("y_accel_meas =", y_accel_meas)

    assert np.isclose(y_accel_est[0], y_accel_meas[0], 1.0e-6)
    assert np.isclose(y_accel_est[1], y_accel_meas[1], 1.0e-6)
    assert np.isclose(y_accel_est[2], y_accel_meas[2], 1.0e-6)

    #--------------- Random motion. ---------------#
    phi = np.deg2rad(-5.0)
    theta = np.deg2rad(-10.0)
    p, q, r = 0.3, -0.1, -0.2
    Va = 40.0
    u = np.array([Va, p, q, r])
    x = np.array([phi, theta])
    y_accel_est = ekf.get_y_accel_est(x, u)
    # print("\ny_accel_est =", y_accel_est)

    # Estimator assumes u = Va*cos(theta), v = 0, w = Va*sin(theta)
    u = Va * np.cos(theta)
    v = 0.0
    w = Va * np.sin(theta)
    x = np.array([u, v, w, phi, theta, p, q, r])
    # Estimator assumes udot = 0, vdot = 0, wdot = 0
    xdot = np.array([0.0, 0.0, 0.0])

    y_accel_meas = get_y_accel_meas(x, xdot)
    # print("y_accel_meas =", y_accel_meas)

    assert np.isclose(y_accel_est[0], y_accel_meas[0], 1.0e-6)
    assert np.isclose(y_accel_est[1], y_accel_meas[1], 1.0e-6)
    assert np.isclose(y_accel_est[2], y_accel_meas[2], 1.0e-6)

def test_wrap_phi_and_theta():
    pass
    


# def test_get_vfpa():

    # # Straight up (VFPA = 90.0)
    # vN, vE, vD = 0.0, 0.0, -100.0 # positive z-axis is down
    # v_ned = np.array([vN, vE, vD])
    # vfpa = kin.get_vfpa(v_ned)
    # assert vfpa == approx(np.deg2rad(90.0), abs=1.0e-6)

    # # Straight down (VFPA = -90.0)
    # vN, vE, vD = 0.0, 0.0, 100.0 # positive z-axis is down
    # v_ned = np.array([vN, vE, vD])
    # vfpa = kin.get_vfpa(v_ned)
    # assert np.isclose(vfpa, np.deg2rad(-90.0), 1.0e-6)

    # # Make sure it returns NaN for zero velocity
    # v_ned = np.array([0.0, 0.0, 0.0])
    # vfpa = kin.get_vfpa(v_ned)
    # assert np.isnan(vfpa)

# def solve_f_equals_ma_explicit(x_vec, mass_props, F, M):

#     u = x_vec[3]
#     v = x_vec[4]
#     w = x_vec[5]
#     roll = x_vec[6]
#     pitch = x_vec[7]
#     yaw = x_vec[8]
#     p = x_vec[9]
#     q = x_vec[10]
#     r = x_vec[11]

#     m = mass_props.get_mass()
#     J = mass_props.get_inertia_matrix()
#     Jinv = mass_props.get_inertia_matrix_inv()
#     gamma_vec = mass_props.get_gamma_vec()

#     Jy = J[1,1]
#     gamma = gamma_vec[0]
#     gamma1 = gamma_vec[1]
#     gamma2 = gamma_vec[2]
#     gamma3 = gamma_vec[3]
#     gamma4 = gamma_vec[4]
#     gamma5 = gamma_vec[5]
#     gamma6 = gamma_vec[6]
#     gamma7 = gamma_vec[7]
#     gamma8 = gamma_vec[8]

#     cr = np.cos(roll)
#     sr = np.sin(roll)
#     cp = np.cos(pitch)
#     sp = np.sin(pitch)
#     tp = np.tan(pitch)
#     cy = np.cos(yaw)
#     sy = np.sin(yaw)

#     pn_dot = cp*cy*u + (sr*sp*cy-cr*sy)*v + (cr*sp*cy+sr*sy)*w
#     pe_dot = cp*sy*u + (sr*sp*sy+cr*cy)*v + (cr*sp*sy-sr*cy)*w
#     pd_dot = -sp*u + sr*cp*v + cr*cp*w

#     udot = F[0] / m - q*w + r*v
#     vdot = F[1] / m - r*u + p*w
#     wdot = F[2] / m - p*v + q*u

#     roll_dot = p + (q*sr + r*cr)*tp
#     pitch_dot = q*cr - r*sr
#     yaw_dot = (q*sr + r*cr)/cp

#     l, m, n = M

#     pdot = gamma1 * p * q - gamma2 * q * r + gamma3 * l + gamma4 * n
#     qdot = gamma5 * p * r - gamma6 * (p**2 - r**2) + m / Jy
#     rdot = gamma7 * p * q - gamma1 * q * r + gamma4 * l + gamma8 * n

#     return np.array([
#         pn_dot, pe_dot, pd_dot,
#         udot, vdot, wdot,
#         roll_dot, pitch_dot, yaw_dot,
#         pdot, qdot, rdot,
#     ])

# def test_solve_f_equals_ma():

#     mass = 10.0
#     Jx = 1.0
#     Jy = 2.0
#     Jz = 3.0
#     Jxz = 4.0
#     mass_props = mp.MassProperties(mass, Jx, Jy, Jz, Jxz)

#     roll = np.deg2rad(5.0)
#     pitch = np.deg2rad(10.0)

#     u = 50.0 # [m/s]
#     v = 10.0 # [m/s]
#     w = -5.0 # [m/s]
#     p = -0.3 # [rad/s]
#     q = 0.2 # [rad/s]
#     r = -0.1 # [rad/s]
#     uvw = np.array([u, v, w])
#     pqr = np.array([p, q, r])

#     F = np.array([6.0, -7.0, 8.0])
#     M = np.array([-11.0, 12.0, -13.0])


#     x_vec = np.array([0.0, 0.0, 0.0, u, v, w, roll, pitch, 0.0, p, q, r])
#     x_ac = kin.NonlinearKinematicState(x_vec)

#     x_dot_explicit = solve_f_equals_ma_explicit(x_vec, mass_props, F, M)
#     x_dot_implicit = x_ac.solve_f_equals_ma(mass_props, F, M)

#     print("\nx_dot_implicit =\n", x_dot_implicit)
#     print("x_dot_explicit =\n", x_dot_explicit)

#     assert x_dot_implicit[0] == approx(x_dot_explicit[0], abs=1.0e-6)
#     assert x_dot_implicit[1] == approx(x_dot_explicit[1], abs=1.0e-6)
#     assert x_dot_implicit[2] == approx(x_dot_explicit[2], abs=1.0e-6)
#     assert x_dot_implicit[3] == approx(x_dot_explicit[3], abs=1.0e-6)
#     assert x_dot_implicit[4] == approx(x_dot_explicit[4], abs=1.0e-6)
#     assert x_dot_implicit[5] == approx(x_dot_explicit[5], abs=1.0e-6)
#     assert x_dot_implicit[6] == approx(x_dot_explicit[6], abs=1.0e-6)
#     assert x_dot_implicit[7] == approx(x_dot_explicit[7], abs=1.0e-6)
#     assert x_dot_implicit[8] == approx(x_dot_explicit[8], abs=1.0e-6)
#     assert x_dot_implicit[9] == approx(x_dot_explicit[9], abs=1.0e-6)
#     assert x_dot_implicit[10] == approx(x_dot_explicit[10], abs=1.0e-6)
#     assert x_dot_implicit[11] == approx(x_dot_explicit[11], abs=1.0e-6)

