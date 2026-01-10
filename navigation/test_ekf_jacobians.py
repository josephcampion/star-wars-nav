
import numpy as np

from navigation.pitch_roll_ekf import PitchRollEKF, GRAV_ACCEL_MPS2

# Initialize EKF
phi0 = np.deg2rad(10.0)
theta0 = np.deg2rad(10.0)
x_hat_pred = np.array([phi0, theta0]) # + random_init 
P_pred = np.eye(2)
x_hat_upd = np.array([phi0, theta0]) # + random_init 
P_upd = np.eye(2)
Q = np.eye(2) * 1.e-4
R = np.eye(3) * 1.e-2
T = 0.01 # seconds
ekf = PitchRollEKF(Q, R, T)



def test_J_df_dx():

    x = np.array([0.3, 0.4]) # [phi, theta]^T
    u = np.array([0.0, 0.1, 0.2, -0.3]) # [Va, p, q, r]^T

    J_an = ekf.get_J_df_dx(x, u)
    print("Analytic Jacobian:\n", J_an)

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
    
    x = np.array([0.3, 0.4]) # [phi, theta]^T
    u = np.array([0.0, 0.1, 0.2, -0.3]) # [Va, p, q, r]^T
    n = len(x)
    l = 3 # len(y_accel_est)

    J_an = ekf.get_J_dh_dx(x, u)
    print("Analytic Jacobian:\n", J_an)

    h = 1.0e-6
    J_num = np.zeros((l, n))
    for i in range(n):
        x_i = x.copy()
        x_i[i] += h
        h_of_x_i = ekf.get_y_accel_est(x_i, u)
        h_of_x = ekf.get_y_accel_est(x, u)
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
