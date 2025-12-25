
import numpy as np
import matplotlib.pyplot as plt

from sensors import GPS

####################################################
    #   Initialize Simulation Parameters
####################################################

Tsim = 1000.0 # seconds
dt = 1.0 # timestep
t0 = 0.0

time = np.arange(t0, Tsim, dt)
nt = len(time)

# Set truth motion for sensor testing
pn_truth = 10.0 * np.sin(3.e-2*time)
pe_truth = 20.0 * np.sin(2.e-2*time)
pd_truth = 30.0 * np.sin(1.e-2*time)
# pn_truth = np.ones(nt)

pn_meas = np.zeros(nt)
pe_meas = np.zeros(nt)
pd_meas = np.zeros(nt)

gps_error_n = np.zeros(nt)
gps_error_e = np.zeros(nt)
gps_error_d = np.zeros(nt)

####################################################
    #   Run Simulation of Sensor Models
####################################################

# Make sensor models
gps = GPS(0.21, 0.21, 0.40, 1.0/1100.0, 1.0)

for i in range(nt):
    pos_ned_truth = [
        pn_truth[i],
        pe_truth[i],
        pd_truth[i]
    ]

    gps.update_gps_error()
    pos_ned_meas = gps.get_gps_data(pos_ned_truth)
    gps_error = gps.get_gps_error()

    pn_meas[i], pe_meas[i], pd_meas[i] = pos_ned_meas
    gps_error_n[i], gps_error_e[i], gps_error_d[i] = gps_error

####################################################
            #   Plot Results
####################################################

_, axs = plt.subplots(3,1)

axs[0].plot(time, pn_truth, label='Truth')
axs[0].grid(True)
axs[0].plot(time, pn_meas, label='Sensor', linestyle=':')
# axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('[m]')
axs[0].set_title('GPS Position vs Truth')
axs[0].legend()

axs[1].plot(time, pe_truth, label='Truth')
axs[1].grid(True)
axs[1].plot(time, pe_meas, label='Sensor', linestyle=':')
# axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('[m]')
# axs[1].set_title('Angular Velocity Sensor vs Truth')
axs[1].legend()

axs[2].plot(time, pd_truth, label='Truth')
axs[2].grid(True)
axs[2].plot(time, pd_meas, label='Sensor', linestyle=':')
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('[m]')
# axs[2].set_title('Angular Velocity Sensor vs Truth')
axs[2].legend()

# Plot GPS Error
_, axs = plt.subplots(3,1)

axs[0].plot(time, gps_error_n)
axs[0].grid(True)
# axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('[m]')
axs[0].set_title('GPS Error')

axs[1].plot(time, gps_error_e)
axs[1].grid(True)
# axs[1].set_xlabel('Time [s]')
axs[1].set_ylabel('[m]')
# axs[1].set_title('Specific Acceleration Sensor vs Truth')

axs[2].plot(time, gps_error_d)
axs[2].grid(True)
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('[m]')
# axs[2].set_title('Specific Acceleration Sensor vs Truth')

plt.show()
