
import matplotlib.pyplot as plt

class Plotter:
    # X = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
    # i = [0, 1,   2,  3, 4, 5, 6,   7,     8,   9,10,11]
    # Xdot = [vn, ve, vd, udot, vdot, wdot, phi_dot, theta_dot, psi_dot, pdot, qdot, rdot]
    # i =    [0,  1,  2,  3,    4,    5,    6,       7,         8,       9,    10,   11]
    # Linear (small angle) assumptions:
        # phi_dot ~= p
        # theta_dot ~= q
        # psi_dot ~= r
    def __init__(self, t, X, Xdot):
        self._t = t
        self._X = X
        self._Xdot = Xdot

    def plot_sim(self):
        # TODO: plot control inputs (de, dt, da, dr)
        self.plot_lon_axis()
        self.plot_lat_axis()
        self.plot_lin_axis()
        self.plot_ang_axis()
        self.plot_all_states()
        self.plot_all_derivatives()
        plt.show()
        return

    def plot_lon_axis(self):
        fig_lon, ax_lon = plt.subplots(4,2)

        ax_lon[0,0].plot(self._t, self._X[:,5])
        ax_lon[1,0].plot(self._t, self._Xdot[:,5])
        ax_lon[2,0].plot(self._t, self._X[:,3])
        ax_lon[3,0].plot(self._t, self._Xdot[:,3])

        ax_lon[0,1].plot(self._t, self._X[:,5])
        ax_lon[1,1].plot(self._t, self._X[:,7])
        ax_lon[2,1].plot(self._t, self._X[:,10])
        ax_lon[3,1].plot(self._t, self._Xdot[:,10])

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

        ax_lat[0,0].plot(self._t, self._X[:,4])
        ax_lat[1,0].plot(self._t, self._Xdot[:,4])
        ax_lat[2,0].plot(self._t, self._X[:,11])
        ax_lat[3,0].plot(self._t, self._Xdot[:,11])

        ax_lat[0,1].plot(self._t, self._X[:,4])
        ax_lat[1,1].plot(self._t, self._X[:,6])
        ax_lat[2,1].plot(self._t, self._X[:,9])
        ax_lat[3,1].plot(self._t, self._Xdot[:,9])

        ax_lat[0,0].set_title(r"$v$")
        ax_lat[1,0].set_title(r"$\dot{v}$")
        ax_lat[2,0].set_title(r"$r$")
        ax_lat[3,0].set_title(r"$\dot{r}$")

        ax_lat[0,1].set_title(r"$\beta\approx \frac{v}{V_0}$")
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
        ax_lin[1,0].plot(self._t, self._X[:,3])
        ax_lin[2,0].plot(self._t, self._Xdot[:,3])

        ax_lin[0,1].plot(self._t, self._X[:,1])
        ax_lin[1,1].plot(self._t, self._X[:,4])
        ax_lin[2,1].plot(self._t, self._Xdot[:,4])

        ax_lin[0,2].plot(self._t, self._X[:,2])
        ax_lin[1,2].plot(self._t, self._X[:,5])
        ax_lin[2,2].plot(self._t, self._Xdot[:,5])

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

        ax_lin[0,0].plot(self._t, self._X[:,6])
        ax_lin[1,0].plot(self._t, self._X[:,9])
        ax_lin[2,0].plot(self._t, self._Xdot[:,9])

        ax_lin[0,1].plot(self._t, self._X[:,7])
        ax_lin[1,1].plot(self._t, self._X[:,10])
        ax_lin[2,1].plot(self._t, self._Xdot[:,10])

        ax_lin[0,2].plot(self._t, self._X[:,8])
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
    
    def plot_all_states(self):
        fig_all, ax_all = plt.subplots(4,3)

        ax_all[0,0].plot(self._t, self._X[:,0])
        ax_all[0,1].plot(self._t, self._X[:,1])
        ax_all[0,2].plot(self._t, self._X[:,2])

        ax_all[1,0].plot(self._t, self._X[:,3])
        ax_all[1,1].plot(self._t, self._X[:,4])
        ax_all[1,2].plot(self._t, self._X[:,5])

        ax_all[2,0].plot(self._t, self._X[:,6])
        ax_all[2,1].plot(self._t, self._X[:,7])
        ax_all[2,2].plot(self._t, self._X[:,8])

        ax_all[3,0].plot(self._t, self._X[:,9])
        ax_all[3,1].plot(self._t, self._X[:,10])
        ax_all[3,2].plot(self._t, self._X[:,11])

        ax_all[0,0].set_title(r"$p_n$")
        ax_all[0,1].set_title(r"$p_e$")
        ax_all[0,2].set_title(r"$p_d$")

        ax_all[1,0].set_title(r"$u$")
        ax_all[1,1].set_title(r"$v$")
        ax_all[1,2].set_title(r"$w$")

        ax_all[2,0].set_title(r"$\phi$")
        ax_all[2,1].set_title(r"$\theta$")
        ax_all[2,2].set_title(r"$\psi$")

        ax_all[3,0].set_title(r"$p$")
        ax_all[3,1].set_title(r"$q$")
        ax_all[3,2].set_title(r"$r$")

        for i in range(3):
            ax_all[0,i].set_ylabel("[m]")
            ax_all[1,i].set_ylabel("[m/s]")
            ax_all[2,i].set_ylabel("[rad]")
            ax_all[3,i].set_ylabel("[rad/s]")

        # Add gridlines
        for i in range(4):
            for j in range(3):
                ax_all[i,j].grid(True)
        
        # Label bottom rows x-axes
        for j in range(3):
            ax_all[3,j].set_xlabel("Time [s]")

        return
    
    def plot_all_derivatives(self):
        fig_all, ax_all = plt.subplots(4,3)

        ax_all[0,0].plot(self._t, self._Xdot[:,0])
        ax_all[0,1].plot(self._t, self._Xdot[:,1])
        ax_all[0,2].plot(self._t, self._Xdot[:,2])

        ax_all[1,0].plot(self._t, self._Xdot[:,3])
        ax_all[1,1].plot(self._t, self._Xdot[:,4])
        ax_all[1,2].plot(self._t, self._Xdot[:,5])

        ax_all[2,0].plot(self._t, self._Xdot[:,6])
        ax_all[2,1].plot(self._t, self._Xdot[:,7])
        ax_all[2,2].plot(self._t, self._Xdot[:,8])

        ax_all[3,0].plot(self._t, self._Xdot[:,9])
        ax_all[3,1].plot(self._t, self._Xdot[:,10])
        ax_all[3,2].plot(self._t, self._Xdot[:,11])

        ax_all[0,0].set_title(r"$v_n$")
        ax_all[0,1].set_title(r"$v_e$")
        ax_all[0,2].set_title(r"$v_d$")

        ax_all[1,0].set_title(r"$\dot{u}$")
        ax_all[1,1].set_title(r"$\dot{v}$")
        ax_all[1,2].set_title(r"$\dot{w}$")

        ax_all[2,0].set_title(r"$p\approx\dot{\phi}$")
        ax_all[2,1].set_title(r"$q\approx\dot{\theta}$")
        ax_all[2,2].set_title(r"$r=\approx\dot{\psi}$")

        ax_all[3,0].set_title(r"$\dot{p}$")
        ax_all[3,1].set_title(r"$\dot{q}$")
        ax_all[3,2].set_title(r"$\dot{r}$")

        for i in range(3):
            ax_all[0,i].set_ylabel("[m/s]")
            ax_all[1,i].set_ylabel("[m/s^2]")
            ax_all[2,i].set_ylabel("[rad/s]")
            ax_all[3,i].set_ylabel("[rad/s^2]")

        # Add gridlines
        for i in range(4):
            for j in range(3):
                ax_all[i,j].grid(True)
        
        # Label bottom rows x-axes
        for j in range(3):
            ax_all[3,j].set_xlabel("Time [s]")

        return