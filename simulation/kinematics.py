
import numpy as np
import simulation.rotations as rt
import models.mass_props as mp

class KinematicState:
    def __init__(self, init_conds=np.zeros(12)):
        self._pn = init_conds[0]
        self._pe = init_conds[1]
        self._pd = init_conds[2]
        self._u = init_conds[3]
        self._v = init_conds[4]
        self._w = init_conds[5]
        # TODO: Replace Euler angles with quaternions
        self._phi = init_conds[6]
        self._theta = init_conds[7]
        self._psi = init_conds[8]
        self._p = init_conds[9]
        self._q = init_conds[10]
        self._r = init_conds[11]

    def get_state(self):
        return np.array([
            self._pn,
            self._pe,
            self._pd,
            self._u,
            self._v,
            self._w,
            self._phi,
            self._theta,
            self._psi,
            self._p,
            self._q,
            self._r,
        ])
    
    def set_state(self, x_vec):
        self._pn = x_vec[0]
        self._pe = x_vec[1]
        self._pd = x_vec[2]
        self._u = x_vec[3]
        self._v = x_vec[4]
        self._w = x_vec[5]
        self._phi = x_vec[6]
        self._theta = x_vec[7]
        self._psi = x_vec[8]
        self._p = x_vec[9]
        self._q = x_vec[10]
        self._r = x_vec[11]

    # TODO: Output airspeed, course, vertical flight path angle

####################################################
# TODO: Make state derivatives given forces & moments
####################################################

class LinearKinematicState(KinematicState):
    def __init__(self, init_conds=np.zeros(12)):
        super().__init__(init_conds=init_conds)

    def get_lon_state(self):
        return np.array([
            self._u,
            self._w,
            self._q,
            self._theta,
        ])
    
    def get_lat_state(self):
        return np.array([
            self._v,
            self._p,
            self._phi,
            self._r,
        ])

class NonlinearKinematicState(KinematicState):
    def __init__(self, init_conds=np.zeros(12)):
        super().__init__(init_conds=init_conds)

    # For these simulations, assuming NED is inertial frame.
    def solve_f_equals_ma(self, mass_props, F, M):

        roll = self._phi
        pitch = self._theta
        m = mass_props.get_mass()
        J = mass_props.get_inertia_matrix()
        Jinv = mass_props.get_inertia_matrix_inv()

        uvw = np.array([self._u, self._v, self._w])
        pqr = np.array([self._p, self._q, self._r])

        # pned_dot = DCM_bod2ned * uvw == uvw_ned
        DCM_ned2bod = rt.get_dcm_ned2bod(roll, pitch, self._psi)
        DCM_bod2ned = DCM_ned2bod.T
        pned_dot = DCM_bod2ned @ uvw

        # F = m * a = m * v_ned_dot = m * (uvw_dot + cross(pqr, uvw))
        uvw_dot = F / m - np.cross(pqr, uvw)

        A_rpy_dot_to_pqr = rt.get_pqr_to_rpy_dot(roll, pitch)
        rpy_dot = A_rpy_dot_to_pqr @ pqr

        # M = dH_dt = J*pqr_dot + cross(pqr, J*pqr) 
        pqr_dot = Jinv @ (M - np.cross(pqr, J @ pqr))

        return np.concatenate([pned_dot, uvw_dot, rpy_dot, pqr_dot])

