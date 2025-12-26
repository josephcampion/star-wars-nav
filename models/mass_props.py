
import numpy as np

class InertiaTerms():
    def __init__(self, Jx, Jy, Jz, Jxz):
        self._Jx = Jx
        self._Jy = Jy
        self._Jz = Jz
        self._Jxz = Jxz
    
    def get_inertia_matrix(self):
        return np.array([
            [self._Jx, 0.0, -self._Jxz],
            [0.0, self._Jy, 0.0],
            [-self._Jxz, 0.0, self._Jz]
        ])
    
    def get_inertia_matrix_inv(self):

        gamma = self._Jx * self._Jz - self._Jxz**2

        return np.array([
            [self._Jz / gamma, 0.0, self._Jxz / gamma],
            [0.0, 1.0 / self._Jy, 0.0],
            [self._Jxz / gamma, 0.0, self._Jx / gamma]
        ])

    def get_gamma_vec(self):

        Jx = self._Jx
        Jy = self._Jy
        Jz = self._Jz
        Jxz = self._Jxz

        gamma = Jx * Jz - Jxz**2
        gamma1 = Jxz * (Jx - Jy + Jz) / gamma
        gamma2 = (Jz * (Jz - Jy) + Jxz**2) / gamma
        gamma3 = Jz / gamma
        gamma4 = Jxz / gamma
        gamma5 = (Jz - Jx) / Jy
        gamma6 = Jxz / Jy
        gamma7 = ((Jx - Jy) * Jx + Jxz**2) / gamma
        gamma8 = Jx / gamma
        
        return np.array([gamma, gamma1, gamma2, gamma3, \
            gamma4, gamma5, gamma6, gamma7, gamma8])

class MassProperties:
    def __init__(self, mass, Jx, Jy, Jz, Jxz):
        self._m = mass
        self._inertia_terms = InertiaTerms(Jx, Jy, Jz, Jxz)

    def get_mass(self):
        return self._m

    def get_inertia_matrix(self):
        return self._inertia_terms.get_inertia_matrix()

    def get_inertia_matrix_inv(self):
        return self._inertia_terms.get_inertia_matrix_inv()

    def get_gamma_vec(self):
        return self._inertia_terms.get_gamma_vec()
