
import numpy as np

from navigation.pos_hdg_wnd_ekf import PosHdgWndEKF, GRAV_ACCEL_MPS2
from models.sensors import get_y_accel_meas

# Initialize EKF
pn0 = 0.1
pe0 = -0.2
vg0 = 0.3
chi0 = -0.4
wn0 = 0.5
we0 = -0.6
psi0 = 0.7
x0 = np.array([pn0, pe0, vg0, chi0, wn0, we0, psi0])
x_hat_pred = x0.copy() # + random_init 
P_pred = np.eye(len(x0)) * 1.e-2
x_hat_upd = x_hat_pred.copy()
P_upd = P_pred.copy()
Q = np.eye(len(x0)) * 1.e-4
R = np.eye(6) * 1.e-2
T = 0.01 # seconds
ekf = PosHdgWndEKF(Q, R, T)

# # TODO: Move this to test_sensors.py
# def test_get_y_accel_meas():

#     # x = [u, v, w, phi, theta, p, q, r]
#     # xdot = [udot, vdot, wdot]

#     #--------------- No motion, no pitch or roll. ---------------#
#     x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     xdot = np.array([0.0, 0.0, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)

#     assert np.isclose(y_accel_meas[0], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_meas[1], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_meas[2], -GRAV_ACCEL_MPS2, 1.0e-6)

#     #--------------- No motion, pitched nose up. ---------------#
#     x = np.array([0.0, 0.0, 0.0, 0.0, np.deg2rad(45.0), 0.0, 0.0, 0.0])
#     xdot = np.array([0.0, 0.0, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)

#     ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
#     assert np.isclose(y_accel_meas[0], ans, 1.0e-6)
#     assert np.isclose(y_accel_meas[1], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

#     #--------------- No motion, rolled right. ---------------#
#     x = np.array([0.0, 0.0, 0.0, np.deg2rad(45.0), 0.0, 0.0, 0.0, 0.0])
#     xdot = np.array([0.0, 0.0, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)

#     ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
#     assert np.isclose(y_accel_meas[0], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_meas[1], -ans, 1.0e-6)
#     assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

#     #--------------- Full steam ahead (udot>0). ---------------#
#     x = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     udot_ans = 10.0
#     xdot = np.array([udot_ans, 0.0, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)

#     ans = GRAV_ACCEL_MPS2
#     assert np.isclose(y_accel_meas[0], udot_ans, 1.0e-6)
#     assert np.isclose(y_accel_meas[1], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

#     #--------------- Starboard acceleration (vdot>0). ---------------#
#     x = np.array([0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     vdot_ans = 10.0
#     xdot = np.array([0.0, vdot_ans, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)
#     ans = GRAV_ACCEL_MPS2
#     assert np.isclose(y_accel_meas[0], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_meas[1], vdot_ans, 1.0e-6)
#     assert np.isclose(y_accel_meas[2], -ans, 1.0e-6)

# def test_get_y_accel_est():

#     # Compare with get_y_accel_meas (under conditions with no wind or acceleration).
#     # Estimator assumes udot=vdot=wdot=0, alpha=theta, beta=0.

#     # x = [phi, theta]^T
#     # u = [Va, p, q, r]^T

#     #--------------- No motion, no pitch or roll. ---------------#
#     x = np.array([0.0, 0.0])
#     xdot = np.array([0.0, 0.0, 0.0, 0.0])

#     y_accel_est = ekf.get_y_accel_est(x, xdot)

#     assert np.isclose(y_accel_est[0], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_est[1], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_est[2], -GRAV_ACCEL_MPS2, 1.0e-6)

#     #--------------- No motion, pitched nose up. ---------------#
#     x = np.array([0.0, np.deg2rad(45.0)])
#     u = np.array([10.0, 0.0, 0.0, 0.0])

#     y_accel_est = ekf.get_y_accel_est(x, u)

#     ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
#     assert np.isclose(y_accel_est[0], ans, 1.0e-6)
#     assert np.isclose(y_accel_est[1], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_est[2], -ans, 1.0e-6)

#     #--------------- No motion, rolled right. ---------------#
#     x = np.array([np.deg2rad(45.0), 0.0])
#     u = np.array([10.0, 0.0, 0.0, 0.0])

#     y_accel_est = ekf.get_y_accel_est(x, u)

#     ans = GRAV_ACCEL_MPS2 * np.sqrt(2.0) / 2.0
#     assert np.isclose(y_accel_est[0], 0.0, 1.0e-6)
#     assert np.isclose(y_accel_est[1], -ans, 1.0e-6)
#     assert np.isclose(y_accel_est[2], -ans, 1.0e-6)

#     #--------------- Random motion. ---------------#
#     phi = np.deg2rad(10.0)
#     theta = np.deg2rad(20.0)
#     p, q, r = 0.1, -0.2, 0.3
#     Va = 25.0
#     u = np.array([Va, p, q, r])
#     x = np.array([phi, theta])
#     y_accel_est = ekf.get_y_accel_est(x, u)
#     # print("\ny_accel_est =", y_accel_est)

#     # Estimator assumes u = Va*cos(theta), v = 0, w = Va*sin(theta)
#     u = Va * np.cos(theta)
#     v = 0.0
#     w = Va * np.sin(theta)
#     x = np.array([u, v, w, phi, theta, p, q, r])
#     # Estimator assumes udot = 0, vdot = 0, wdot = 0
#     xdot = np.array([0.0, 0.0, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)
#     # print("y_accel_meas =", y_accel_meas)

#     assert np.isclose(y_accel_est[0], y_accel_meas[0], 1.0e-6)
#     assert np.isclose(y_accel_est[1], y_accel_meas[1], 1.0e-6)
#     assert np.isclose(y_accel_est[2], y_accel_meas[2], 1.0e-6)

#     #--------------- Random motion. ---------------#
#     phi = np.deg2rad(-5.0)
#     theta = np.deg2rad(-10.0)
#     p, q, r = 0.3, -0.1, -0.2
#     Va = 40.0
#     u = np.array([Va, p, q, r])
#     x = np.array([phi, theta])
#     y_accel_est = ekf.get_y_accel_est(x, u)
#     # print("\ny_accel_est =", y_accel_est)

#     # Estimator assumes u = Va*cos(theta), v = 0, w = Va*sin(theta)
#     u = Va * np.cos(theta)
#     v = 0.0
#     w = Va * np.sin(theta)
#     x = np.array([u, v, w, phi, theta, p, q, r])
#     # Estimator assumes udot = 0, vdot = 0, wdot = 0
#     xdot = np.array([0.0, 0.0, 0.0])

#     y_accel_meas = get_y_accel_meas(x, xdot)
#     # print("y_accel_meas =", y_accel_meas)

#     assert np.isclose(y_accel_est[0], y_accel_meas[0], 1.0e-6)
#     assert np.isclose(y_accel_est[1], y_accel_meas[1], 1.0e-6)
#     assert np.isclose(y_accel_est[2], y_accel_meas[2], 1.0e-6)

def test_J_df_dx():

    # x = [pn, pe, vg, chi, wn, we, psi]^T
    x = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
    # u = [Va, q, r, phi, theta]^T
    u = np.array([-0.1, 0.2, -0.3, 0.4, -0.5])

    J_an = ekf.get_J_df_dx(x, u)
    print("\nAnalytic Jacobian:\n", J_an)

    n = len(x)
    h = 1.0e-6
    J_num = np.zeros((n, n))
    for i in range(n):
        x_i = x.copy()
        x_i[i] += h
        f_of_x_i = ekf.get_x_dot(x_i, u)
        f_of_x = ekf.get_x_dot(x, u)
        print("f_of_x_i:\n", f_of_x_i)
        print("f_of_x:\n", f_of_x)
        J_num[:,i] = (f_of_x_i - f_of_x) / h
        
    print("Numerical Jacobian:\n", J_num)
    print("Error:\n", J_num - J_an)

    assert np.isclose(J_num[0,0], J_an[0,0], 1.0e-6)
    assert np.isclose(J_num[0,1], J_an[0,1], 1.0e-6)
    assert np.isclose(J_num[1,0], J_an[1,0], 1.0e-6)
    assert np.isclose(J_num[1,1], J_an[1,1], 1.0e-6)

    return

def test_J_dh_dx():
    
    # x = [pn, pe, vg, chi, wn, we, psi]^T
    x = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
    # u = [Va, q, r, phi, theta]^T
    u = np.array([-0.1, 0.2, -0.3, 0.4, -0.5])
    n = len(x)
    l = 6 # len(y_gps_est)

    J_an = ekf.get_J_dh_dx(x, u)
    print("Analytic Jacobian:\n", J_an)

    h = 1.0e-6
    J_num = np.zeros((l, n))
    for i in range(n):
        x_i = x.copy()
        x_i[i] += h
        h_of_x_i = ekf.get_y_gps_est(x_i, u)
        h_of_x = ekf.get_y_gps_est(x, u)
        print("h_of_x_i:\n", h_of_x_i)
        print("h_of_x:\n", h_of_x)
        print("J_num[:,i] = ", J_num[:,i])
        J_num[:,i] = (h_of_x_i - h_of_x) / h
        

    print("Numerical Jacobian:\n", J_num)
    print("Error:\n", J_num - J_an)

    assert np.isclose(J_num[0,0], J_an[0,0], 1.0e-5)
    assert np.isclose(J_num[0,1], J_an[0,1], 1.0e-5)
    assert np.isclose(J_num[1,0], J_an[1,0], 1.0e-5)
    assert np.isclose(J_num[1,1], J_an[1,1], 1.0e-5)
    assert np.isclose(J_num[2,0], J_an[2,0], 1.0e-5)
    assert np.isclose(J_num[2,1], J_an[2,1], 1.0e-5)

    return
