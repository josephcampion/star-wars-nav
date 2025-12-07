
import matplotlib.pyplot as plt

class Plotter:
    # X = [pn, pe, pd, phi, theta, psi, u, v, w, p, q, r]
    # i = [0, 1,   2,  3,   4,     5,   6, 7, 8, 9,10,11]
    # Xdot = [vn, ve, vd, phi_dot, theta_dot, psi_dot, udot, vdot, wdot, pdot, qdot, rdot]
    # i =    [0,  1,  2,  3,       4,         5,       6,    7,    8,    9,    10,   11]
    # Linear (small angle) assumptions:
        # phi_dot ~= p
        # theta_dot ~= q
        # psi_dot ~= r
    # Xddot = [etc.]
    def __init__(self, t, X, Xdot, Xddot):
        self._t = t
        self._X = X
        self._Xdot = Xdot
        self._Xddot = Xddot

    def plot_sim(self):
        self.plot_lon_axis()
        self.plot_lat_axis()
        self.plot_lin_axis()
        self.plot_ang_axis()
        plt.show()
        return

    def plot_lon_axis(self):
        fig_lon, ax_lon = plt.subplots(4,2)

        ax_lon[0,0].plot(self._t, self._X[:,8]) # w
        ax_lon[1,0].plot(self._t, self._Xdot[:,8]) # wdot
        ax_lon[2,0].plot(self._t, self._X[:,6]) # u
        ax_lon[3,0].plot(self._t, self._Xdot[:,6]) # udot

        ax_lon[0,1].plot(self._t, self._X[:,8]) # alpha ~= w/V0
        ax_lon[1,1].plot(self._t, self._X[:,4]) # theta
        ax_lon[2,1].plot(self._t, self._X[:,10]) # q
        ax_lon[3,1].plot(self._t, self._Xdot[:,10]) # qdot

        ax_lon[0,0].set_title(r"$w$")
        ax_lon[1,0].set_title(r"$\dot{w}$")
        ax_lon[2,0].set_title(r"$u$")
        ax_lon[3,0].set_title(r"$\dot{u}$")

        ax_lon[0,1].set_title(r"$\alpha\approx \frac{w}{V_0}$")
        ax_lon[1,1].set_title(r"$\theta$")
        ax_lon[2,1].set_title(r"$q\approx\dot{\theta}$")
        ax_lon[3,1].set_title(r"$\dot{q}$")

        ax_lon[0,0].set_ylabel("[m/s]")
        ax_lon[1,0].set_ylabel("[m/s^2]")
        ax_lon[2,0].set_ylabel("[m/s]")
        ax_lon[3,0].set_ylabel("[m/s^2]")

        ax_lon[0,1].set_ylabel("[rad]")
        ax_lon[1,1].set_ylabel("[rad]")
        ax_lon[2,1].set_ylabel("[rad/s]")
        ax_lon[3,1].set_ylabel("[rad/s^2]")

        # Add gridlines
        for i in range(4):
            for j in range(2):
                ax_lon[i,j].grid(True)
        
        # Label bottom rows x-axes
        ax_lon[3,0].set_xlabel("Time [s]")
        ax_lon[3,1].set_xlabel("Time [s]")

        return
    
    def plot_lat_axis(self):

        fig_lat, ax_lat = plt.subplots(4,2)

        ax_lat[0,0].plot(self._t, self._X[:,7])
        ax_lat[1,0].plot(self._t, self._Xdot[:,7])
        ax_lat[2,0].plot(self._t, self._X[:,11])
        ax_lat[3,0].plot(self._t, self._Xdot[:,11])

        ax_lat[0,1].plot(self._t, self._X[:,7])
        ax_lat[1,1].plot(self._t, self._X[:,3])
        ax_lat[2,1].plot(self._t, self._X[:,9])
        ax_lat[3,1].plot(self._t, self._Xdot[:,9])

        ax_lat[0,0].set_title(r"$v$")
        ax_lat[1,0].set_title(r"$\dot{v}$")
        ax_lat[2,0].set_title(r"$r$")
        ax_lat[3,0].set_title(r"$\dot{r}$")

        ax_lat[0,1].set_title(r"$\beta\approx \frac{w}{V_0}$")
        ax_lat[1,1].set_title(r"$\phi$")
        ax_lat[2,1].set_title(r"$p\approx\dot{\phi}$")
        ax_lat[3,1].set_title(r"$\dot{p}$")

        ax_lat[0,0].set_ylabel("[m/s]")
        ax_lat[1,0].set_ylabel("[m/s^2]")
        ax_lat[2,0].set_ylabel("[m/s]")
        ax_lat[3,0].set_ylabel("[m/s^2]")

        ax_lat[0,1].set_ylabel("[rad]")
        ax_lat[1,1].set_ylabel("[rad]")
        ax_lat[2,1].set_ylabel("[rad/s]")
        ax_lat[3,1].set_ylabel("[rad/s^2]")

        # Add gridlines
        for i in range(4):
            for j in range(2):
                ax_lat[i,j].grid(True)
        
        # Label bottom rows x-axes
        ax_lat[3,0].set_xlabel("Time [s]")
        ax_lat[3,1].set_xlabel("Time [s]")

        return
    
    def plot_lin_axis(self): # TODO: Add derived states, like vn

        fig_lat, ax_lin = plt.subplots(3,3)

        ax_lin[0,0].plot(self._t, self._X[:,0])
        ax_lin[1,0].plot(self._t, self._X[:,6])
        ax_lin[2,0].plot(self._t, self._Xdot[:,6])

        ax_lin[0,1].plot(self._t, self._X[:,1])
        ax_lin[1,1].plot(self._t, self._X[:,7])
        ax_lin[2,1].plot(self._t, self._Xdot[:,7])

        ax_lin[0,2].plot(self._t, self._X[:,2])
        ax_lin[1,2].plot(self._t, self._X[:,8])
        ax_lin[2,2].plot(self._t, self._Xdot[:,8])

        ax_lin[0,0].set_title(r"$p_n$")
        ax_lin[1,0].set_title(r"$u$")
        ax_lin[2,0].set_title(r"$\dot{u}$")

        ax_lin[0,1].set_title(r"$p_e$")
        ax_lin[1,1].set_title(r"$v$")
        ax_lin[2,1].set_title(r"$\dot{v}$")

        ax_lin[0,2].set_title(r"$p_d$")
        ax_lin[1,2].set_title(r"$w$")
        ax_lin[2,2].set_title(r"$\dot{w}$")

        ax_lin[0,0].set_ylabel("[m]")
        ax_lin[1,0].set_ylabel("[m/s]")
        ax_lin[2,0].set_ylabel("[m/s^2]")

        ax_lin[0,1].set_ylabel("[m]")
        ax_lin[1,1].set_ylabel("[m/s]")
        ax_lin[2,1].set_ylabel("[m/s^2]")

        ax_lin[0,2].set_ylabel("[m]")
        ax_lin[1,2].set_ylabel("[m/s]")
        ax_lin[2,2].set_ylabel("[m/s^2]")
        
        # Add gridlines
        for i in range(3):
            for j in range(3):
                ax_lin[i,j].grid(True)
        
        # Label bottom rows x-axes
        for j in range(3):
            ax_lin[2,j].set_xlabel("Time [s]")
        
        return
    
    def plot_ang_axis(self): # TODO: Add derived states, like course

        fig_lat, ax_lin = plt.subplots(3,3)

        ax_lin[0,0].plot(self._t, self._X[:,3])
        ax_lin[1,0].plot(self._t, self._X[:,9])
        ax_lin[2,0].plot(self._t, self._Xdot[:,9])

        ax_lin[0,1].plot(self._t, self._X[:,4])
        ax_lin[1,1].plot(self._t, self._X[:,10])
        ax_lin[2,1].plot(self._t, self._Xdot[:,10])

        ax_lin[0,2].plot(self._t, self._X[:,5])
        ax_lin[1,2].plot(self._t, self._X[:,11])
        ax_lin[2,2].plot(self._t, self._Xdot[:,11])

        ax_lin[0,0].set_title(r"$\phi$")
        ax_lin[1,0].set_title(r"$p\approx\dot{\phi}$")
        ax_lin[2,0].set_title(r"$\dot{p}$")

        ax_lin[0,1].set_title(r"$\theta$")
        ax_lin[1,1].set_title(r"$q\approx\dot{\theta}$")
        ax_lin[2,1].set_title(r"$\dot{q}$")

        ax_lin[0,2].set_title(r"$\psi$")
        ax_lin[1,2].set_title(r"$r\approx\dot{\psi}$")
        ax_lin[2,2].set_title(r"$\dot{r}$")

        ax_lin[0,0].set_ylabel("[rad]")
        ax_lin[1,0].set_ylabel("[rad/s]")
        ax_lin[2,0].set_ylabel("[rad/s^2]")

        ax_lin[0,1].set_ylabel("[rad]")
        ax_lin[1,1].set_ylabel("[rad/s]")
        ax_lin[2,1].set_ylabel("[rad/s^2]")

        ax_lin[0,2].set_ylabel("[rad]")
        ax_lin[1,2].set_ylabel("[rad/s]")
        ax_lin[2,2].set_ylabel("[rad/s^2]")
        
        # Add gridlines
        for i in range(3):
            for j in range(3):
                ax_lin[i,j].grid(True)
        
        # Label bottom rows x-axes
        for j in range(3):
            ax_lin[2,j].set_xlabel("Time [s]")
        
        return