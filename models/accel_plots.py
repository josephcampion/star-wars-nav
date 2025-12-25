
import sensors
import numpy as np
import matplotlib.pyplot as plt

####################################################
    #   Initialize Simulation Parameters
####################################################

Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Set truth motion for sensor testing
ax_truth = 1.0 * np.sin(3*time)
ay_truth = 2.0 * np.sin(2*time)
az_truth = 3.0 * np.sin(1*time)

x_accel_meas = np.zeros(nt)
y_accel_meas = np.zeros(nt)
z_accel_meas = np.zeros(nt)

####################################################
    #   Run Simulation of Sensor Models
####################################################

# Make sensor models
x_accel = sensors.Accelerometer(0.05, 0.2)
y_accel = sensors.Accelerometer(0.05, 0.2)
z_accel = sensors.Accelerometer(0.05, 0.2)

for i in range(nt):
    x_accel_meas[i] = x_accel.get_sensor_data(ax_truth[i])
    y_accel_meas[i] = y_accel.get_sensor_data(ay_truth[i])
    z_accel_meas[i] = z_accel.get_sensor_data(az_truth[i])

####################################################
            #   Plot Results
####################################################

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
