
import numpy as np

# TODO: Make this a function of altitude?
AIR_DENSITY_KGM3 = 1.2682 # [kg*m^3]

class MassProperties:
    def __init__(self, mass, inertia_list):

        if len(inertia_list) == 6:
            Jx = inertia_list[0]
            Jy = inertia_list[1]
            Jz = inertia_list[2]
            Jxy = inertia_list[3]
            Jxz = inertia_list[4]
            Jyz = inertia_list[5]
        elif len(inertia_list) == 4: # assuming Jxy and Jyz are 0
            Jx = inertia_list[0]
            Jy = inertia_list[1]
            Jz = inertia_list[2]
            Jxy = 0.0
            Jxz = inertia_list[3]
            Jyz = 0.0
        else:
            print("Inertia list needs to be of len 4 or 6.")

        # TODO: Replace with cross product/matrix calc
        # or do that with a unit test...
        Gamma = Jx * Jz - Jxz**2
        Gamma1 = Jxz * (Jx - Jy + Jz) / Gamma
        Gamma2 = (Jz * (Jz - Jy) + Jxz**2) / Gamma
        Gamma3 = Jz / Gamma
        Gamma4 = Jxz / Gamma
        Gamma5 = (Jz - Jx) / Jy
        Gamma6 = Jxz / Jy
        Gamma7 = ((Jx - Jy) * Jx + Jxz**2) / Gamma
        Gamma8 = Jx / Gamma

        self._m = mass
        self._Jx = Jx
        self._Jy = Jy
        self._Jz = Jz
        self._Jxy = Jxy
        self._Jxz = Jxz
        self._Jyz = Jyz
        self._Gamma = Gamma
        self._Gamma1 = Gamma1
        self._Gamma2 = Gamma2
        self._Gamma3 = Gamma3
        self._Gamma4 = Gamma4
        self._Gamma5 = Gamma5
        self._Gamma6 = Gamma6
        self._Gamma7 = Gamma7
        self._Gamma8 = Gamma8

# end MassProperties

class AerodynamicCoefficients:
    def __init__(self, aero_coeffs_list):

        #----------------- Longitudinal ----------------#
        C_L_0 = 0.28
        C_D_0 = 0.03
        C_m_0 = -0.02338

        C_L_alpha = 3.45
        C_D_alpha = 0.30
        C_m_alpha = -0.38

        C_L_q = 0.0
        C_D_q = 0.0
        C_m_q = -3.6

        C_L_delta_e = -0.36
        C_D_delta_e = 0.0
        C_m_delta_e = -0.5

        # C_D_p = 0.0437

        #----------------- Lateral -----------------#

        C_Y_0 = 0.0
        C_l_0 = 0.0
        C_n_0 = 0.0

        C_Y_beta = -0.98
        C_l_beta = -0.12
        C_n_beta = 0.25

        C_Y_p = 0.0
        C_l_p = -0.26
        C_n_p = 0.022

        C_Y_r = 0.0
        C_l_r = 0.14
        C_n_r = -0.35

        C_Y_delta_a = 0.0
        C_l_delta_a = 0.08
        C_n_delta_a = 0.06

        C_Y_delta_r = -0.17
        C_l_delta_r = 0.105
        C_n_delta_r = -0.032

# end AerodynamicCoefficients
        
class EngineProperties:
    def __init__(self, engine_params_list):

        S_prop = 0.2027 # [m^2]
        k_motor = 80.0 # [-]
        k_T_p = 0 # [-]
        k_Omega = 0.0 # [-]
        e = 0.9 # [-]

        C_prop = 1.0
        M = 50.0
        alpha_0 = 0.4712
        eta = 0.1592

# class LinearStateSpaceModel:
#     def __init__(self, mass_props, engine_props, aero_coeffs):