
import numpy as np
import matplotlib.pyplot as plt

from models.sensors import InertialMeasurementUnit

####################################################
    #   Initialize Simulation Parameters
####################################################

Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Set truth motion for sensor testing
p_truth = 1.0 * np.sin(3*time)
q_truth = 2.0 * np.sin(2*time)
r_truth = 3.0 * np.sin(1*time)

# ang_vel_truth = [
#     p_truth,
#     q_truth,
#     r_truth
# ]

ax_truth = 1.0 * np.sin(3*time)
ay_truth = 2.0 * np.sin(2*time)
az_truth = 3.0 * np.sin(1*time)
# p_truth = np.ones(nt)

# TODO: Re-do this with sensed vs. real acceleration (i.e., include gravity)
x_gyro_meas = np.zeros(nt)
y_gyro_meas = np.zeros(nt)
z_gyro_meas = np.zeros(nt)

x_accel_meas = np.zeros(nt)
y_accel_meas = np.zeros(nt)
z_accel_meas = np.zeros(nt)

####################################################
    #   Run Simulation of Sensor Models
####################################################

# Make sensor models
imu = InertialMeasurementUnit(0.1, 0.1, 0.05, 0.2)

for i in range(nt):
    ang_vel_truth = [
        p_truth[i],
        q_truth[i],
        r_truth[i]
    ]
    sens_accel_truth = [
        ax_truth[i],
        ay_truth[i],
        az_truth[i]
    ]

    ang_vel_meas = imu.get_ang_vel_meas(ang_vel_truth)
    sens_accel_meas = imu.get_sens_accel_meas(sens_accel_truth)

    x_gyro_meas[i], y_gyro_meas[i], z_gyro_meas[i] = ang_vel_meas
    x_accel_meas[i], y_accel_meas[i], z_accel_meas[i] = sens_accel_meas

####################################################
    #   Plot Results (TODO: Consolidate with gyro_accel_test.py)
####################################################

_, axs = plt.subplots(3,1)

axs[0].plot(time, p_truth, label='Truth')
axs[0].grid(True)
axs[0].plot(time, x_gyro_meas, label='Sensor', linestyle=':')
# axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('[rad/s]')
axs[0].set_title('Angular Velocity Sensor vs Truth')
axs[0].legend()

axs[1].plot(time, q_truth, label='Truth')
axs[1].grid(True)
axs[1].plot(time, y_gyro_meas, label='Sensor', linestyle=':')
# axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('[rad/s]')
# axs[1].set_title('Angular Velocity Sensor vs Truth')
axs[1].legend()

axs[2].plot(time, r_truth, label='Truth')
axs[2].grid(True)
axs[2].plot(time, z_gyro_meas, label='Sensor', linestyle=':')
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('[rad/s]')
# axs[2].set_title('Angular Velocity Sensor vs Truth')
axs[2].legend()

_, axs = plt.subplots(3,1)

axs[0].plot(time, ax_truth, label='Truth')
axs[0].grid(True)
axs[0].plot(time, x_accel_meas, label='Sensor', linestyle=':')
# axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('[m/s^2]')
axs[0].set_title('Specific Acceleration Sensor vs Truth')
axs[0].legend()

axs[1].plot(time, ay_truth, label='Truth')
axs[1].grid(True)
axs[1].plot(time, y_accel_meas, label='Sensor', linestyle=':')
# axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('[m/s^2]')
# axs[1].set_title('Specific Acceleration Sensor vs Truth')
axs[1].legend()

axs[2].plot(time, az_truth, label='Truth')
axs[2].grid(True)
axs[2].plot(time, z_accel_meas, label='Sensor', linestyle=':')
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('[m/s^2]')
# axs[2].set_title('Specific Acceleration Sensor vs Truth')
axs[2].legend()

plt.show()
