
import numpy as np
import models.aerosonde_uav as ac

# TODO: Make this a function of altitude.
AIR_DENSITY_KGM3 = 1.2682 # [kg*m^3]
        
class EngineProperties:
    def __init__(self, engine_params_list):
        if len(engine_params_list) != 5:
            print("engine_params_list should be length 5.")
        self._S_prop = engine_params_list[0] # propeller area
        self._k_motor = engine_params_list[1] # motor constant
        self._k_T_p = engine_params_list[2] # torque constant
        self._k_Omega = engine_params_list[3] # angular velocity constant
        self._C_prop = engine_params_list[4] # thrust coefficient
        # TODO: Double check where M, e and eta should be in vehicle model.
        # self._e = engine_params_list[5] # ?
        # self._M = engine_params_list[6] # moment of inertia?
        # self._eta = engine_params_list[7] # efficiency?

    def get_thrust(self, Va, delta_t):

        S_prop = self._S_prop
        C_prop = self._C_prop
        k_motor = self._k_motor

        # TODO: Make this a function of altitude.
        rho = AIR_DENSITY_KGM3

        F_thrust = 0.5 * rho * S_prop * C_prop * ((k_motor * delta_t)**2 - Va**2)
                
        return F_thrust

    def get_gyro_torque(self, delta_t):

        k_T_p = self._k_T_p
        k_Omega = self._k_Omega

        T_prop = -k_T_p * (k_Omega * delta_t)**2

        return T_prop

    def get_prop_force_and_moment(self, Va, delta_t):

        F_thrust = self.get_thrust(Va, delta_t)
        T_prop = self.get_gyro_torque(delta_t)

        F_prop = np.array([F_thrust, 0.0, 0.0])
        M_prop = np.array([T_prop, 0.0, 0.0])

        return F_prop, M_prop

if __name__ == "__main__":

    engine_props = EngineProperties([ac.S_prop, \
        ac.k_motor, ac.k_T_p, ac.k_Omega, ac.C_prop])

    F_prop, M_prop = engine_props.get_prop_force_and_moment(10.0, 1.0)

    print("F_prop = ", F_prop)
    print("M_prop = ", M_prop)
