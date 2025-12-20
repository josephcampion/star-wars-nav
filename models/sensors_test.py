
# import sensors
import numpy as np
import matplotlib.pyplot as plt

# Initialize sim parameters
Tsim = 10.0 # seconds
dt = 0.01 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# TODO: Get these from parameters file
sensor_bias = 0.1 # [m/s]
noise_std = 0.1

x_truth = 2.0 * np.sin(3*time)
# x_truth = np.ones(nt)
x_sensor = np.zeros(nt)

for i in range(nt):
    x_sensor[i] = x_truth[i] + sensor_bias + np.random.normal(0, noise_std)

plt.plot(time, x_sensor, label='Sensor')
plt.plot(time, x_truth, label='Truth')
plt.grid(True)
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.title('Angular Velocity Sensor vs Truth')
plt.legend()
plt.show()
