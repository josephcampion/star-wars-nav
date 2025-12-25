
import numpy as np
import models.aerosonde_uav as ac
from models.utils import GammaInertiaTerms

# TODO: Make this a function of altitude.
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

        self._m = mass
        self._Jx = Jx
        self._Jy = Jy
        self._Jz = Jz
        self._Jxy = Jxy
        self._Jxz = Jxz
        self._Jyz = Jyz
        self._Gamma = GammaInertiaTerms(Jx, Jy, Jz, Jxz)

# end MassProperties

class AerodynamicCoefficients:
    def __init__(self, aero_coeffs_list):
        if len(aero_coeffs_list) != 30:
            print("aero_coeffs_list should be length 30.")
        #----------------- Longitudinal ----------------#
        self._C_L_0 = aero_coeffs_list[0]
        self._C_D_0 =  aero_coeffs_list[1]
        self._C_m_0 =  aero_coeffs_list[2]
        self._C_L_alpha =  aero_coeffs_list[3]
        self._C_D_alpha =  aero_coeffs_list[4]
        self._C_m_alpha =  aero_coeffs_list[5]
        self._C_L_q =  aero_coeffs_list[6]
        self._C_D_q =  aero_coeffs_list[7]
        self._C_m_q =  aero_coeffs_list[8]
        self._C_L_delta_e =  aero_coeffs_list[9]
        self._C_D_delta_e =  aero_coeffs_list[10]
        self._C_m_delta_e =  aero_coeffs_list[11]
        # self._C_D_p = 0.0437
        #----------------- Lateral -----------------#
        self._C_Y_0 = aero_coeffs_list[12]
        self._C_l_0 = aero_coeffs_list[13]
        self._C_n_0 = aero_coeffs_list[14]
        self._C_Y_beta = aero_coeffs_list[15]
        self._C_l_beta = aero_coeffs_list[16]
        self._C_n_beta = aero_coeffs_list[17]
        self._C_Y_p = aero_coeffs_list[18]
        self._C_l_p = aero_coeffs_list[19]
        self._C_n_p = aero_coeffs_list[20]
        self._C_Y_r = aero_coeffs_list[21]
        self._C_l_r = aero_coeffs_list[22]
        self._C_n_r = aero_coeffs_list[23]
        self._C_Y_delta_a = aero_coeffs_list[24]
        self._C_l_delta_a = aero_coeffs_list[25]
        self._C_n_delta_a = aero_coeffs_list[26]
        self._C_Y_delta_r = aero_coeffs_list[27]
        self._C_l_delta_r = aero_coeffs_list[28]
        self._C_n_delta_r = aero_coeffs_list[29]

# end AerodynamicCoefficients
        
class EngineProperties:
    def __init__(self, engine_params_list):
        if len(engine_params_list) != 8:
            print("engine_params_list should be length 8.")
        self._S_prop = engine_params_list[0]
        self._k_motor = engine_params_list[1]
        self._k_T_p = engine_params_list[2]
        self._k_Omega = engine_params_list[3]
        self._e = engine_params_list[4]
        self._C_prop = engine_params_list[5]
        self._M = engine_params_list[6]
        self._eta = engine_params_list[7]


class LinearStateSpaceModel:
    def __init__(self, mass_props, engine_props, aero_coeffs, trim_conds):
        self._mass_props = mass_props
        self._engine_props = engine_props
        self._aero_coeffs = aero_coeffs
        self._trim_conds = trim_conds
        

if __name__ == "__main__":

    mass_props = MassProperties(ac.m, [ac.Jx, ac.Jy, ac.Jz, ac.Jxz])

    engine_props = EngineProperties([
        ac.S_prop, ac.k_motor, ac.k_T_p, ac.k_Omega, ac.e, \
        ac.C_prop, ac.M, ac.eta
    ])

    aero_coeffs = AerodynamicCoefficients([
        ac.C_L_0, ac.C_D_0, ac.C_m_0, \
        ac.C_L_alpha, ac.C_D_alpha, ac.C_m_alpha, \
        ac.C_L_q, ac.C_D_q, ac.C_m_q, \
        ac.C_L_delta_e, ac.C_D_delta_e, ac.C_m_delta_e, \
        ac.C_Y_0, ac.C_l_0, ac.C_n_0, \
        ac.C_Y_beta, ac.C_l_beta, ac.C_n_beta, \
        ac.C_Y_p, ac.C_l_p, ac.C_n_p, \
        ac.C_Y_r, ac.C_l_r, ac.C_n_r, \
        ac.C_Y_delta_a, ac.C_l_delta_a, ac.C_n_delta_a, \
        ac.C_Y_delta_r, ac.C_l_delta_r, ac.C_n_delta_r
    ])

    # print(dir(mass_props))
    # print(dir(engine_props))
    # print(dir(aero_coeffs))
