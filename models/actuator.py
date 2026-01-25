
import numpy as np
import control as ct
import matplotlib.pyplot as plt
from flight_controls.utils import plot_bode, mag2db

# Closed-loop actuator control
f_n_actuator = 8.0 # [Hz]
wn_actuator = 2 * np.pi * f_n_actuator # [rad/s]
zeta_actuator = 0.8

tf_actuator = ct.tf([wn_actuator**2], [1.0, 2 * zeta_actuator * wn_actuator, wn_actuator**2])

if __name__ == "__main__":

    print(tf_actuator)

    mag, phase_rad, omega = ct.bode(tf_actuator, plot=False)

    fig, ax = plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega)
    fig.suptitle(r'$G_{actuator}(s)=\frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$')

    plt.show()
