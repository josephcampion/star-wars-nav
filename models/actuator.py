
import numpy as np
import control as ct
import matplotlib.pyplot as plt
from flight_controls.utils import plot_bode, mag2db

# Closed-loop actuator control
fn_actuator = 5.0 # [Hz]
wn_actuator = 2 * np.pi * fn_actuator # [rad/s]
zeta_actuator = 0.707

tf_actuator = ct.tf([wn_actuator**2], [1.0, 2 * zeta_actuator * wn_actuator, wn_actuator**2])

if __name__ == "__main__":

    print(tf_actuator)

    mag, phase_rad, omega = ct.bode(tf_actuator, plot=False)

    fig, ax = plt.subplots(2,1)
    plot_bode(mag2db(mag), np.rad2deg(phase_rad), omega, ax)
    fig.suptitle(r'$G_{actuator}(s)=\frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$')

    Tsim = 1.0
    response = ct.step_response(tf_actuator , T=Tsim)
    t = response.time
    actuator_t = response.y[0,0,:] # actuator response

    fig, ax = plt.subplots()
    ax.plot(t, actuator_t, label="Actuator")
    ax.set_title(r'$G_{actuator}(s)=\frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$')
    ax.set_ylabel('[rad]')
    ax.set_xlabel('Time [s]')
    ax.grid(True)    
    plt.show()
