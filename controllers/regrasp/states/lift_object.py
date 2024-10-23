# imports
import numpy as np
from controllers.base.baseState import *
from .fsm_config import *

# define state
class LiftObject(BaseState):
    def __init__(self):
        self.name = "LiftObject"
        self.enabled = 0
        self.start_time = 0

    def enter(self, GP):
        print(self.name)
        self.enabled = 1
        # get initial time
        self.start_time = GP.time()

        self.current_des =  GP.gr_data.get_q(GP.gr_data.all_idxs)
        self.current_tau_ff = GP.gr_data.get_tau_ff(GP.gr_data.all_idxs)


    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):

        # stay in this state
        next_state = self.name

        # get current time
        cur_time = GP.time()

        current_des = GP.gr_data.get_q_des(GP.gr_data.all_idxs)

        #desired grasp position #changing this doesn't do anything??
        q_des_hold = self.current_des.copy()
        q_des_hold[1] = -0.5
        q_des_hold[5] = -0.5
        

        #lifting kp 
        # kp_grasp = np.array([3.0, 8.0, 2.5, 8, 2.5, 8.0, 2.5, 2.5, 8])
       

        # # go to desired grasp positions
        ratio = (cur_time - self.start_time)/fsm_params.times['lift']
        GP.gr_data.set_q_des(GP.gr_data.all_idxs, ratio*q_des_hold + (1-ratio)*self.current_des)

        
        # # GP.gr_data.set_q_des(GP.gr_data.all_idxs,   fsm_params.q_des_lift)
        # GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
        # GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
        # GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
        # GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)

        # # # set wrist cartesian position
        # GP.gr_data.kinematics['base_des']['p'] = fsm_params.base_pos_default
        # GP.gr_data.kinematics['base_des']['R'] = fsm_params.base_R_default


        # # # hold at desired grasp positions #original
        # GP.gr_data.set_q_des(GP.gr_data.all_idxs,   current_des)
        # GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
        # GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
        # GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
        # GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)

        # hold at desired grasp positions 
        
        #if no feedforward torques were set, lift up object with fsm param q desired
        if np.sum(np.abs(self.current_tau_ff)) < 1e-4:
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   current_des)
            GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
            GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)
        #if feedforward torques were calculated from close along normals, lift with given torques
        else: 
            kp_grasp = fsm_params.kp_grasp.copy()
            kp_grasp[2:4] = [0,0]
            kp_grasp[6:8] = [0,0]
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   current_des)
            GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  self.current_tau_ff)
            GP.gr_data.set_kp(GP.gr_data.all_idxs,      kp_grasp)
            GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)


        # set wrist cartesian position
        ratio = (cur_time - self.start_time)/fsm_params.times['lift']
        GP.gr_data.kinematics['base_des']['p'] = ratio*fsm_params.base_pos_hold + (1-ratio)*fsm_params.base_pos_default
        # TODO: interpolate rotations?
        GP.gr_data.kinematics['base_des']['R'] = fsm_params.base_R_hold #ratio*fsm_params.base_R_hold + (1-ratio)*fsm_params.base_R_default

        # state transition to holding object
        if (cur_time-self.start_time) > fsm_params.times['lift']:
            next_state = "HoldObject"

        # check for manual state transition to reset
        if GP.char_in=='R' or GP.char_in=='r':
            next_state = "Reset"

        return next_state