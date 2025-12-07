
import matplotlib.pyplot as plt

class Plotter:
    # [pn, pe, pd, phi, theta, psi, u, v, w, p, q, r]
    def __init__(self, t, X, Xdot, Xddot):
        self._t = t
        self._X = X
        self._Xdot = Xdot
        self._Xddot = Xddot

    def plot_sim(self):

        _, axs = plt.subplots(3,1)

        axs[0].plot(self._t, self._X[:,1]) # , linewidth=2) # , label=r"$\sigma(t) = \frac{1}{1 + e^{-t}}$")
        axs[1].plot(self._t, self._Xdot[:,1]) # , linewidth=2)
        axs[2].plot(self._t, self._Xddot[:,1]) # , linewidth=2)

        axs[0].set_ylabel("Pos. [m]")
        axs[1].set_ylabel("Vel. [m/s]")
        axs[2].set_ylabel("Accel. [ms/^2]")

        axs[2].set_xlabel("Time [s]")

        axs[0].grid(True)
        axs[1].grid(True)
        axs[2].grid(True)

        plt.show()

        return