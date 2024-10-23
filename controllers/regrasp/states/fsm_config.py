# define config variables for all states here

# imports
import numpy as np

# general params class to add attributes to for each state
class FSMparams:
    pass

fsm_params = FSMparams()

# NOTE: (from gripper data)
# w_idxs = [0] # joint idx for wrist
# l_idxs = [1,2,3,4] # joint idxs for left finger
# r_idxs = [5,6,7,8] # joint idxs for right finger

# default gains for fingers and wrist
fsm_params.kp_default = np.array([3.0, 8.0, 2.5, 2.5, 2.5, 8.0, 2.5, 2.5, 2.5])
fsm_params.kd_default = np.array([0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])

# initial state of fingers, wrist, object
fsm_params.q_des_default = np.array([0.0, 0.0, 0.4, -0.4, -0.0, 0.0, -0.4, 0.4, 0.0])

fsm_params.base_pos_default = np.array([0.0, 0.0, 0.05])
fsm_params.base_R_default = np.eye(3)

fsm_params.obj_pos_default = np.array([0.18, 0.0, 0.031])
fsm_params.obj_R_default = np.eye(3)

# finger pose for grasping, lifting, holding
# fsm_params.q_des_grasp = np.array([0.0, 0.0, 0.2, -0.8, 0.4, 0.0, -0.2, 0.8, -0.4])
# fsm_params.q_des_grasp = np.array([0.0, 0.0, 0.5, -1.4, -0.9, 0.0, -0.5, 1.4, 0.9]) # from old FSM
#for ellipsoid
fsm_params.q_des_grasp = np.array([0.0, 0.0, 0.5, -2.0, 1.5, 0.0, -0.5, 2.0, -1.5]) # from old FSM

fsm_params.q_des_grasp_ellipsoid = np.array([0.0, 0.0, 0.5, -1.4, -0.9, 0.0, -0.5, 1.4, 0.9]) # from old FSM

# finger gains for grasping, lifting, holding (if not defaults?)
fsm_params.kp_grasp = np.array([3.0, 8.0, 2.5, 2.5, 2.5, 8.0, 2.5, 2.5, 2.5])
fsm_params.kd_grasp = np.array([0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])

# wrist pose for grasping
fsm_params.base_pos_grasp = np.array([0.0, 0.0, 0.05])
fsm_params.base_R_grasp = np.eye(3)

# grasping trajectory parameters
fsm_params.grasp_traj_time = 1.0
fsm_params.grasp_attempt_limit = 5

# regrasping check parameters
fsm_params.regrasp_inside_thresh = 0.090 # in m
fsm_params.grasp_thresh = 0.2 # rad
fsm_params.palm_power_thresh = 0.045 # m
fsm_params.palm_pinch_thresh = 0.11 # m
fsm_params.grasp_eq_vel_thresh = 0.2 # 0.05 m/s
fsm_params.grasp_force_thresh =  0.5 # N # TODO: replace with normal_force_threshold?
fsm_params.grasp_tip_collision_thresh = 0.02 # in meters
fsm_params.antipodal_thresh = 20.0 # in degrees

# wrist pose for holding
fsm_params.base_pos_hold = np.array([0.0, 0.0, 0.25])
fsm_params.base_R_hold = np.eye(3)


# dict of trajectory times for each state
fsm_params.times = {'wait':     1.0,
                    'regrasp':  1.0,
                    'grasp':    5.0,
                    'lift':     2.0,
                    'hold':     5.0}

fsm_params.l1 = 0.050
fsm_params.l2 = 0.040
fsm_params.l3 = 0.0365
fsm_params.finger_spacing = 0.113


# fsm_params.l_wrist_gripper = 0.06
fsm_params.t_gripper_fingerMidpoint = [0.1155, 0.0, 0.0111]

# from old FSM:
# #Regrasp thresholds
# grasp_params.psi_lim = 0.25 # roughly 20deg, might be too wide
# grasp_params.r_sensor = 0.01 # add sensor radius and some extra clearance to radius estimate
# grasp_params.r_eps = 0.01
# grasp_params.r_bodge = 0.01
# grasp_params.angle = 15*3.14/180 #angle offset for grasp closure trigger