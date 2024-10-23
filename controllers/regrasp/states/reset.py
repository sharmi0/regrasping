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

    def enter(self, GP):
        print(self.name)
        self.enabled = 1

    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):

        # directly set finger positions
        GP.mj_data.qpos[7:16] = fsm_params.q_des_default
        GP.mj_data.qvel[7:16] = np.zeros((9,))

        # directly set wrist position
        GP.mj_data.mocap_pos = fsm_params.base_pos_default
        base_quat_default = np.zeros((4,))
        mj.mju_mat2Quat(base_quat_default, fsm_params.base_R_default.flatten())
        GP.mj_data.mocap_quat = base_quat_default

        # directly set block position
        body_id = GP.mj_model.body("cube").id
        # GP.mj_data.joint("cube_joint").qpos[:3 ]= fsm_params.obj_pos_default + (1 - 2 * np.random.rand(3))*np.array([0.05, 0.02, 0.0])
        GP.mj_data.joint("cube_joint").qpos[:3 ]= fsm_params.obj_pos_default + np.array([0.045, 0.0, 0.0])

        obj_quat_default = np.zeros((4,))
        mj.mju_mat2Quat(obj_quat_default, fsm_params.obj_R_default.flatten())
        GP.mj_data.joint("cube_joint").qpos[3:] = obj_quat_default

        # go to waiting state after reset
        print(GP.mj_data.joint("cube_joint").qpos)
        next_state = "Waiting"

        return next_state