
import numpy as np
import matplotlib.pyplot as plt
import control as ct

def get_u_to_y_tf(A, B, C, D, u_ind, y_ind):

    # B = [B_u0, B_u1]
    # shape needs to be (4,1), not (4,)
    B_u = np.array([[B[i,u_ind]] for i in range(4)])

    # y = C * x
    # yi = Ci * x
    C_y = np.zeros([1,4])
    C_y[0,y_ind] = 1

    D_u_y = np.array([[0.0]]) # shape needs to be (1,1)

    tf_u_to_y = ct.ss2tf(ct.ss(A, B_u, C_y, D_u_y))

    return tf_u_to_y


def plot_lon_step_response(t, x, label="", amplitude=1.0):
    fig, ax = plt.subplots(2,2)

    ax[0,0].plot(t, amplitude * x[0]) # , label=r'$u$')
    ax[0,1].plot(t, amplitude * x[1]) # , label=r'$w$')
    # ax[1,0].plot(t, amplitude * x[2]) # , label=r'$q$')
    # ax[1,1].plot(t, amplitude * x[3]) # , label=r'$\theta$')
    ax[1,0].plot(t, np.rad2deg(amplitude * x[2])) # , label=r'$q$')
    ax[1,1].plot(t, np.rad2deg(amplitude * x[3])) # , label=r'$\theta$')

    ax[0,0].set_title(r'$u$')
    ax[0,1].set_title(r'$w$')
    ax[1,0].set_title(r'$q$')
    ax[1,1].set_title(r'$\theta$')

    for i in range(2):
        for j in range(2):
            ax[i,j].grid(True)
            # ax[i,j].legend()

    ax[0,0].set_ylabel('[kts]')
    # ax[1,0].set_ylabel('[rad(/sec)]')
    ax[1,0].set_ylabel('[deg(/sec)]')

    ax[1,0].set_xlabel('Time [s]')
    ax[1,1].set_xlabel('Time [s]')

    plt.suptitle(label + " Step Response")


####################################################
        #   Frequency Space Utilities
####################################################

def mag2db(mag):
    return 20 * np.log10(mag)


def plot_bode(mag_db, phase_deg, omega, ax, label=""):
    ax[0].semilogx(omega, mag_db, label=label)
    ax[1].semilogx(omega, phase_deg) # , label=label)
    ax[0].set_ylabel('[dB]')
    ax[1].set_ylabel('[deg]')
    # ax[0].set_xlabel('Frequency [rad/s]')
    ax[1].set_xlabel('Frequency [rad/s]')
    for axi in ax:
        axi.minorticks_on()
        axi.grid(which='major', color='k', linestyle='-', linewidth=0.5)
        axi.grid(which='minor', color='0.9', linestyle='-', alpha=0.75)
        axi.grid(True)
    ax[0].legend()

def plot_nichols(mag_db, phase_deg, ax, label=""):
    ax.plot(phase_deg, mag_db, label=label)
    ax.set_ylabel('[dB]')
    ax.set_xlabel('[deg]')
    ct.nichols_grid()  # Adds standard Nichols chart grid
    ax.legend()

def plot_nichols_margins(ax):
    gm_vec = [6, 8, 12]
    pm_vec = [30, 45, 60]
    color_vec = ['r', 'y', 'g']
    for i in range(len(gm_vec)):
        for x0 in [-180]: # , 180]:
            a, b = pm_vec[i], gm_vec[i]  # semi-major and semi-minor axes
            t = np.linspace(0, 2*np.pi, 100)
            y0 = 0.0
            x = x0 + a * np.cos(t)
            y = y0 + b * np.sin(t)
            ax.plot(x, y, '--', color=color_vec[i], linewidth=0.75)

def wrap_phase(phase_deg):
    while phase_deg[0] < -180:
        phase_deg += 360
    while phase_deg[0] > 180:
        phase_deg -= 360
    return phase_deg
