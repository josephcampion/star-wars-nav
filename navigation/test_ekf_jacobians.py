
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


def numerical_jacobian(f, x, u):
    h = 1.0e-6
    J = np.zeros((len(x), len(x)))
    for i in range(len(x)):
        x_i = x.copy()
        x_i[i] += h
        J[:,i] = (f(x_i, u) - f(x, u)) / h
    return J

def test_get_J_df_dx():
    pass

    # x = [u, v, w, phi, theta, p, q, r]
    # xdot = [udot, vdot, wdot]

    #--------------- Starboard acceleration (vdot>0). ---------------#
    
# """ 
# From ChatGPT:
# import numpy as np

# def num_jacobian(f, x, u, eps=1e-6):
#     y0 = f(x, u)
#     J = np.zeros((2, 2))

#     for i in range(2):
#         dx = np.zeros_like(x)
#         dx[i] = eps
#         y1 = f(x + dx, u)
#         J[:, i] = (y1 - y0) / eps

#     return J

# x = np.array([0.3, 0.4])
# u = np.array([0.0, 0.1, 0.2, -0.3])

# J_num = num_jacobian(self.get_x_dot, x, u)
# J_an  = self.get_J_df_dx(x, u)

# print("Numerical:\n", J_num)
# print("Analytic:\n", J_an)
# print("Error:\n", J_an - J_num)
# """


# def test_get_y_accel_est():

#     # Compare with get_y_accel_meas (under conditions with no wind or acceleration).
#     # Estimator assumes udot=vdot=wdot=0, alpha=theta, beta=0.

#     # x = [phi, theta]^T
#     # u = [Va, p, q, r]^T

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
