
import numpy as np
import matplotlib.pyplot as plt

from models.sensors import Gyroscope

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

x_gyro_meas = np.zeros(nt)
y_gyro_meas = np.zeros(nt)
z_gyro_meas = np.zeros(nt)

####################################################
    #   Run Simulation of Sensor Models
####################################################

# Make sensor models
x_gyro = Gyroscope(0.1, 0.1)
y_gyro = Gyroscope(0.1, 0.1)
z_gyro = Gyroscope(0.1, 0.1)

for i in range(nt):
    x_gyro_meas[i] = x_gyro.get_sensor_data(p_truth[i])
    y_gyro_meas[i] = y_gyro.get_sensor_data(q_truth[i])
    z_gyro_meas[i] = z_gyro.get_sensor_data(r_truth[i])

####################################################
            #   Plot Results
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

plt.show()
