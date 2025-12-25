
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
