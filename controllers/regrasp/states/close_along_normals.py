# imports
import numpy as np
import mujoco as mj
from controllers.base.baseState import *
from .fsm_config import *

# define state                           
class CloseAlongNormals(BaseState):
    def __init__(self):
        self.name = "CloseAlongNormals"
        self.enabled = 0

    def enter(self, GP):
        print(self.name)
        self.enabled = 1

    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):

        Rl_cur = GP.gr_data.kinematics['l_dip_force']['R']
        Rr_cur = GP.gr_data.kinematics['r_dip_force']['R']
        R_l_contact = Rl_cur @ GP.gr_data.sensors['l_dip'].T_sensor_contact[0:3,0:3]
        R_r_contact = Rr_cur @ GP.gr_data.sensors['r_dip'].T_sensor_contact[0:3,0:3]
        l_normal = R_l_contact[:,2].flatten()
        r_normal = R_r_contact[:,2].flatten()
        
        Jl_cur = GP.gr_data.kinematics['l_dip_tip']['Jacp']
        Jr_cur = GP.gr_data.kinematics['r_dip_tip']['Jacp']
        
        Fl, taul = self.close_along_normal(-fsm_params.pinch_force,l_normal,Jl_cur) 
        Fr, taur = self.close_along_normal(-fsm_params.pinch_force,r_normal,Jr_cur)
        
        first2links_idxs = [2,3,6,7]
        GP.gr_data.set_kp(first2links_idxs, np.zeros((4,)))
        GP.gr_data.set_kd(first2links_idxs, np.zeros((4,)))


        tau_close = -np.concatenate([taul,taur])
        GP.gr_data.set_tau_ff(first2links_idxs,tau_close)

        # state transition to execute grasp
        next_state = "LiftObject"

        # check for manual state transition to reset
        if GP.char_in=='R' or GP.char_in=='r':
            next_state = "Reset"

        return next_state
    
        ### helper functions ###

    def close_along_normal(self,force,normal,J):
        tau_ff = np.zeros((4,1))
        F = np.zeros((2,1))
        F = force*normal[0:2]

        # convert forces to joint torques (for first two links only)
        J2 = J[0:2,1:3]
        tau_ff = np.matmul(J2.T, F)


        return F, tau_ff