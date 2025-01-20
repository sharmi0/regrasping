# imports
import numpy as np
import mujoco as mj
from controllers.base.baseState import *
from .fsm_config import *

# define state
class Reset(BaseState):
    def __init__(self):
        self.name = "Reset"
        self.enabled = 0
        self.start_time = 0

    def enter(self, GP):
        print(self.name)
        # get initial time
        self.start_time = GP.time()
        self.enabled = 1
        self.q_start = GP.gr_data.get_q(GP.gr_data.all_idxs).copy()


    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):
        # get current time
        cur_time = GP.time()

        # stay in this state
        next_state = self.name

        # this is to make the reset traj slower. 
        # if the target default q's are not close to the current q's and the reset time has not passed, 
        #       follow the trajectory. Otherwise, go straight to the positions.
        #not sure why but when it's np.all, the slow traj works in hardware and not in sim. when it's np.any,
        # the slow traj works in sim but not in hardware
        if (not np.all(np.isclose(self.q_start,fsm_params.q_des_default,rtol=1e-03))) \
            and ((cur_time-self.start_time) < fsm_params.reset_traj_time):
            # follow trajectory to base position
            GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
            GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)

            ratio = (cur_time-self.start_time)/fsm_params.reset_traj_time
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   fsm_params.q_des_default*ratio + self.q_start*(1-ratio))
        #
        else:
            # GP.gr_data.set_q_des(GP.gr_data.all_idxs,   fsm_params.q_des_default)
            GP.gr_data.set_kp(GP.gr_data.all_idxs, np.zeros((9,)))
            GP.gr_data.set_kd(GP.gr_data.all_idxs, np.zeros((9,)))
            GP.mj_data.qpos[7:16] = fsm_params.q_des_default
            GP.mj_data.qvel[7:16] = np.zeros((9,))

            # directly set wrist position
            GP.mj_data.mocap_pos = fsm_params.base_pos_default
            base_quat_default = np.zeros((4,))
            mj.mju_mat2Quat(base_quat_default, fsm_params.base_R_default.flatten())
            GP.mj_data.mocap_quat = base_quat_default

            # directly set block position and radius
            object_radius = 0.01*np.random.rand()+0.03 #randomly pick radius between 0.02m and 0.06m
            object_x_pos = 0.04*np.random.rand()+0.03 #randomly pick radius between 0.03m and 0.7m
            object_y_pos = 0.01*np.random.rand()-0.005 #randomly pick radius between -0.005m an 0.005m

            print(np.array([object_x_pos,object_y_pos,0]))


            # GP.mj_data.joint("cube_joint").qpos[:3 ]= fsm_params.obj_pos_default + (1 - 2 * np.random.rand(3))*np.array([0.05, 0.02, 0.0])
            # GP.mj_data.joint("cube_joint").qpos[:3 ]= fsm_params.obj_pos_default + np.array([0.06, 0.0, 0.0])
            GP.mj_data.joint("cube_joint").qpos[:3 ]= fsm_params.obj_pos_default + np.array([object_x_pos,object_y_pos,0])
            GP.mj_model.geom("object").size[0] = object_radius

            obj_quat_default = np.zeros((4,))
            mj.mju_mat2Quat(obj_quat_default, fsm_params.obj_R_default.flatten())
            GP.mj_data.joint("cube_joint").qpos[3:] = obj_quat_default

            # go to waiting state after reset
            print(GP.mj_data.joint("cube_joint").qpos)
            next_state = "Waiting"

        return next_state