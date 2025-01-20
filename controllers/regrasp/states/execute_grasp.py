# imports
import numpy as np
from controllers.base.baseState import *
from .fsm_config import *

# define state
class ExecuteGrasp(BaseState):
    def __init__(self):
        self.name = "ExecuteGrasp"
        self.enabled = 0
        self.start_time = 0
        self.grasp_attempts = 0

    def enter(self, GP):
        print(self.name)
        self.enabled = 1
        # get initial time
        self.start_time = GP.time()
        # get initial gripper joint angles
        self.q_start = GP.gr_data.get_q(GP.gr_data.all_idxs).copy()
        # increment grasp attempts
        self.grasp_attempts += 1

    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):

        # stay in this state
        next_state = self.name

        # get current time
        cur_time = GP.time()

        # get current finger states
        ql_cur = GP.gr_data.get_q(GP.gr_data.l_idxs)
        qdl_cur = GP.gr_data.get_qd(GP.gr_data.l_idxs)
        pl_cur = GP.gr_data.kinematics['l_dip_tip']['p']
        # Rl_cur = GP.gr_data.kinematics['l_dip_tip']['R']
        Rl_cur = GP.gr_data.kinematics['l_dip_force']['R']
        Jl_cur = GP.gr_data.kinematics['l_dip_tip']['Jacp']

        qr_cur = GP.gr_data.get_q(GP.gr_data.r_idxs)
        qdr_cur = GP.gr_data.get_qd(GP.gr_data.r_idxs)
        pr_cur = GP.gr_data.kinematics['r_dip_tip']['p']
        # Rr_cur = GP.gr_data.kinematics['r_dip_tip']['R']
        Rr_cur = GP.gr_data.kinematics['r_dip_force']['R']
        Jr_cur = GP.gr_data.kinematics['r_dip_tip']['Jacp']

        # calculate fingertip velocities
        vl_cur = Jl_cur @ qdl_cur
        vr_cur = Jr_cur @ qdr_cur

        # retrieve sensor data
        # NOTE: dist[0], dist[2] face forward; dist[1], dist[3] face inward; dist[4] faces outward
        l_dist = GP.gr_data.sensors['l_dip'].dist
        r_dist = GP.gr_data.sensors['r_dip'].dist
        palm_dist = GP.gr_data.sensors['palm'].dist
        l_force = GP.gr_data.sensors['l_dip'].contact_force
        l_angle = GP.gr_data.sensors['l_dip'].contact_angle
        r_force = GP.gr_data.sensors['r_dip'].contact_force
        r_angle = GP.gr_data.sensors['r_dip'].contact_angle

        R_l_contact = Rl_cur @ GP.gr_data.sensors['l_dip'].T_sensor_contact[0:3,0:3]
        R_r_contact = Rr_cur @ GP.gr_data.sensors['r_dip'].T_sensor_contact[0:3,0:3]

        # set wrist cartesian position
        GP.gr_data.kinematics['base_des']['p'] = fsm_params.base_pos_grasp
        GP.gr_data.kinematics['base_des']['R'] = fsm_params.base_R_grasp

        # follow trajectory to desired grasp positions
        GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
        GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
        GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
        GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)
        if ((cur_time-self.start_time) < fsm_params.grasp_traj_time):
            ratio = (cur_time-self.start_time)/fsm_params.grasp_traj_time
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   fsm_params.q_des_grasp*ratio + self.q_start*(1-ratio))
        else:
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   fsm_params.q_des_grasp)

        # check series of conditions to determine if grasp is successful
        obj_inside = (l_dist[1]<fsm_params.regrasp_inside_thresh) or (l_dist[3]<fsm_params.regrasp_inside_thresh) \
                            or (r_dist[1]<fsm_params.regrasp_inside_thresh) or (r_dist[3]<fsm_params.regrasp_inside_thresh) # is object inside fingertips
        obj_power_dist = (palm_dist<=fsm_params.palm_power_thresh) # is palm sensor triggered, close threshold
        obj_pinch_dist = (palm_dist>fsm_params.palm_power_thresh) and (palm_dist<=fsm_params.palm_pinch_thresh) # is palm sensor triggered, far threshold
        tip_eq_vel = ((np.linalg.norm(vl_cur) < fsm_params.grasp_eq_vel_thresh) and (np.linalg.norm(vr_cur) < fsm_params.grasp_eq_vel_thresh)) # are tip velocities near zero
        tip_contact = ((np.abs(l_force[2]) > fsm_params.grasp_force_thresh) and (np.abs(r_force[2]) > fsm_params.grasp_force_thresh)) # are tip forces above threshold

        conditions = [obj_inside, obj_power_dist, obj_pinch_dist, tip_eq_vel, tip_contact]

        if (obj_inside or obj_power_dist or obj_pinch_dist):
            # check for power grasp success
            if ( obj_power_dist and tip_eq_vel and tip_contact ):
                print("Power grasp success.", l_force, 'N')
                next_state = "LiftObject"
            # check for pinch grasp success
            elif ( obj_pinch_dist and tip_eq_vel and tip_contact ):
                # check antipodal angles to declare success
                l_normal = R_l_contact[:,2].flatten()
                r_normal = R_r_contact[:,2].flatten()
                print("l_normal: ", l_normal, " r_normal: ", r_normal)  # SUS, dot product should have normalized vectors first
                antipodal_angle = np.arccos(np.clip(np.dot(l_normal,-1.0*r_normal), -1.0, 1.0))
                if (antipodal_angle < np.deg2rad(fsm_params.antipodal_thresh)):
                    print("Pinch grasp success, angles are antipodal.")
                    next_state = "CloseAlongNormals"
                else:
                    print("Pinch grasp success, but angles are not antipodal. Regrasping.")
                    next_state = "Regrasp"
            # object has been seen, so keep grasping
            else:
                # print("Object seen, grasp not successful yet. Continuing grasp.")
                if obj_inside: # if internal sensors are triggered, close along sensor normals
                    close_force = 1.0
                    # TODO: implement this section, using commented-out helper function below

        # failure conditions

        # check if grasp is taking too long
        if ((cur_time-self.start_time) > fsm_params.times['grasp']):
            print("Exceeded grasp time.")
            # TODO: add reset back in once re-grasp transitions are done
            # print("Resetting.")
            # next_state = "Reset"
            print("Lifting.")
            next_state = "LiftObject"
        # check if the fingertips are touching one another
        if (np.linalg.norm(pl_cur-pr_cur) < fsm_params.grasp_tip_collision_thresh):
            print("Sensors contacted one another, resetting.")
            next_state = "Reset"
        # check if number of allowed grasp attempts has been exceeded
        if (self.grasp_attempts > fsm_params.grasp_attempt_limit):
            print("Too many grasp attempts, resetting.")
            next_state = "Reset"

        # check for manual state transition to reset
        if GP.char_in=='R' or GP.char_in=='r':
            next_state = "Reset"

        if next_state == "LiftObject" or next_state == "Regrasp":
            self.grasp_attempts = 0
        return next_state
