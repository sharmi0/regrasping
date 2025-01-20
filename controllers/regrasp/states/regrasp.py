# imports
import numpy as np
from controllers.base.baseState import *
from .fsm_config import *
from math import *

# define state
class Regrasp(BaseState):
    def __init__(self):
        self.name = "Regrasp"
        self.enabled = 0
        self.start_time = 0
        self.new_q_des_pregrasp = fsm_params.q_des_grasp.copy()
        self.new_q_des_grasp = fsm_params.q_des_grasp.copy()
        

    def enter(self, GP):
        print(self.name)
        self.enabled = 1
        # get initial time
        self.start_time = GP.time()

        # Get contact points and forces
        # self.l_contact_force = GP.gr_data.sensors['l_dip'].contact_force
        # self.r_contact_force = GP.gr_data.sensors['r_dip'].contact_force

        # self.l_contact_angle = GP.gr_data.sensors['l_dip'].contact_angle
        # self.r_contact_angle = GP.gr_data.sensors['r_dip'].contact_angle

        R_sensorL_contactL = GP.gr_data.sensors["l_dip"].T_sensor_contact[0:3, 0:3]
        R_sensorR_contactR = GP.gr_data.sensors["r_dip"].T_sensor_contact[0:3, 0:3]

        T_sensorL_contactL = GP.gr_data.sensors["l_dip"].T_sensor_contact
        T_sensorR_contactR = GP.gr_data.sensors["r_dip"].T_sensor_contact

        
        t_world_sensorL= GP.gr_data.kinematics['l_dip_force']["p"]
        R_world_sensorL = GP.gr_data.kinematics['l_dip_force']["R"]


        T_world_sensorL = np.eye(4)
        T_world_sensorL[0:3, 0:3] = R_world_sensorL
        T_world_sensorL[0:3, 3] = t_world_sensorL

        t_world_sensorR= GP.gr_data.kinematics['r_dip_force']["p"]
        R_world_sensorR= GP.gr_data.kinematics['r_dip_force']["R"]

        T_world_sensorR = np.eye(4)
        T_world_sensorR[0:3, 0:3] = R_world_sensorR
        T_world_sensorR[0:3, 3] = t_world_sensorR

        T_world_contactL = T_world_sensorL @ T_sensorL_contactL
        T_world_contactR = T_world_sensorR @ T_sensorR_contactR

        R_world_contactL = R_world_sensorL @ R_sensorL_contactL
        R_world_contactR = R_world_sensorR @ R_sensorR_contactR

        left_normal = R_world_contactL[:,2]
        right_normal = R_world_contactR[:,2]

        p_world_contactL = T_world_contactL[0:3, 3]
        p_world_contactR = T_world_contactR[0:3, 3]

        #Solve for estimated circle center and radius
        points = [p_world_contactL, p_world_contactR]
        normals = [left_normal, right_normal]
        r_cent , p_cent = self.sphere_estimate(points, normals) #p is in world frame


        print("r: ", r_cent )
        print("p_cent: ", p_cent)

        #Solving everything for left finger
        #Solve for desired contact location in world coordinates
        p_world_contactdesx = p_cent[0] - fsm_params.t_gripper_fingerMidpoint[0] #setting base of gripper as origin 
        p_world_contactdesy = p_cent[1] + r_cent

        print("desired contact location: ", p_world_contactdesx, p_world_contactdesy )

        #Calculate the location of the beginning of link1 in world coordinates
        p_world_link1basex = 0.097 + 0.0185 #from xml file
        p_world_link1basey = 0.0565 #from xml file

        #Solve for desired location of the end of link2/beginning of link3 in world coordinates for pre-grasp
        clearance_pregrasp = 0.010 #m #clearance for pregrasp from object
        clearance_grasp = 0.002  #m # clearance so you're pressing into the object

       
        #Solve for desired pregrasp location of the end of link2/beginning of link3 in world coordinates for grasp
        p_world_link3basedesx_pregrasp = p_world_contactdesx-fsm_params.l3
        p_world_link3basedesy_pregrasp = p_world_contactdesy + fsm_params.sensor_diameter + clearance_pregrasp

        #Solve for desired location of the end of link2/beginning of link3 in world coordinates for grasp
        p_world_link3basedesx_grasp = p_world_contactdesx-fsm_params.l3
        p_world_link3basedesy_grasp = p_world_contactdesy + fsm_params.sensor_diameter - clearance_grasp

        #get pregrasp coordinates and set desired joint angles
        left_q_des_pregrasp, feasible = self.calculate_coordinates("left", p_world_link3basedesx_pregrasp, p_world_link3basedesy_pregrasp, p_world_link1basey)
        if feasible:
            self.new_q_des_pregrasp[1:5] = left_q_des_pregrasp
        right_q_des_pregrasp, feasible = self.calculate_coordinates("right", p_world_link3basedesx_pregrasp, p_world_link3basedesy_pregrasp, p_world_link1basey)
        if feasible:
            self.new_q_des_pregrasp[5:9] = right_q_des_pregrasp    

        #get grasp coordinates and set desired joint angles
        left_q_des_grasp, feasible = self.calculate_coordinates("left", p_world_link3basedesx_grasp, p_world_link3basedesy_grasp, p_world_link1basey)
        if feasible:
             self.new_q_des_grasp[1:5] = left_q_des_grasp
        right_q_des_grasp, feasible = self.calculate_coordinates("right", p_world_link3basedesx_grasp, p_world_link3basedesy_grasp, p_world_link1basey)
        if feasible:
             self.new_q_des_grasp[5:9] = right_q_des_grasp  


    def exit(self, GP):
        self.enabled = 0                      

    def execute(self, GP):
        # GP.paused = not GP.paused
        # stay in this state
        next_state = self.name

        # get current time
        cur_time = GP.time()
        
        #pregrasp
        if cur_time - self.start_time < fsm_params.times['regrasp']:
            # go to desired grasp positions
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   self.new_q_des_pregrasp)
            GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
            GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)

            # set wrist cartesian position
            GP.gr_data.kinematics['base_des']['p'] = fsm_params.base_pos_default
            GP.gr_data.kinematics['base_des']['R'] = fsm_params.base_R_default         

        #grasp
        elif cur_time - self.start_time < fsm_params.times['regrasp'] + fsm_params.times['grasp']:
            # go to desired grasp positions
            GP.gr_data.set_q_des(GP.gr_data.all_idxs,   self.new_q_des_grasp)
            GP.gr_data.set_qd_des(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_tau_ff(GP.gr_data.all_idxs,  np.zeros((9,)))
            GP.gr_data.set_kp(GP.gr_data.all_idxs,      fsm_params.kp_grasp)
            GP.gr_data.set_kd(GP.gr_data.all_idxs,      fsm_params.kd_grasp)

            # set wrist cartesian position
            GP.gr_data.kinematics['base_des']['p'] = fsm_params.base_pos_default
            GP.gr_data.kinematics['base_des']['R'] = fsm_params.base_R_default
        
        else:
            #check if the angles are antipodal
            Rl_cur = GP.gr_data.kinematics['l_dip_force']['R']
            Rr_cur = GP.gr_data.kinematics['r_dip_force']['R']
            R_l_contact = Rl_cur @ GP.gr_data.sensors['l_dip'].T_sensor_contact[0:3,0:3]
            R_r_contact = Rr_cur @ GP.gr_data.sensors['r_dip'].T_sensor_contact[0:3,0:3]
            l_normal = R_l_contact[:,2].flatten()
            r_normal = R_r_contact[:,2].flatten()
            print("l_normal: ", l_normal, " r_normal: ", r_normal)  # SUS, dot product should have normalized vectors first
            antipodal_angle = np.arccos(np.clip(np.dot(l_normal,-1.0*r_normal), -1.0, 1.0))
            if (antipodal_angle < np.deg2rad(fsm_params.antipodal_thresh)):
                print("Angles Are Antipodal After Regrasp")
                next_state = "CloseAlongNormals"
            else:
                print("Angles Are Not Antipodal,  Reset")
                next_state = "Reset"
            

        # check for manual state transition to reset
        if GP.char_in=='R' or GP.char_in=='r':
            next_state = "Reset"


        return next_state


    def sphere_estimate(self, points, normals):
            # points is list of (3,) arrays
            # normals is list of (3,) arrays
            # returns radius and object center
            # inputs and outputs are in the same frame

            nc_1 = normals[0] # first (3,) array
            nc_2 = normals[1] # second (3,) array

            pc_1 = points[0] # first (3,) array
            pc_2 = points[1] # second (3,) array

            Ac = np.zeros((6,4))
            Ac[0:3,0:3] = np.eye(3)
            Ac[3:6,0:3] = np.eye(3)
            Ac[0:3,3] = -nc_1
            Ac[3:6,3] = -nc_2
            bc = np.zeros((6,1))
            bc[0:3,0] = pc_1
            bc[3:6,0] = pc_2
            xc = np.matmul( np.linalg.pinv(Ac), bc)
            p_cent = xc[0:3,0]
            r_cent = xc[3]
            return r_cent, p_cent
    
    def calculate_coordinates(self, finger, p_world_link3basedesx, p_world_link3basedesy, p_world_link1basey):
        #p_world_link3basedesx, p_world_link3basedesy are the base of link3
        feasible = True
        link_des=np.zeros((4,))
        if finger == "right":
            p_world_link1basey = -p_world_link1basey
            p_world_link3basedesy = -p_world_link3basedesy
        #Solve for joint angles based on knowns: l1, l2, l3, desired contact point (r), location of link1 base (s)
        l1 = fsm_params.l1
        l2 = fsm_params.l2
        l3 = fsm_params.l3
        c = np.sqrt(p_world_link3basedesx**2 + (p_world_link1basey-p_world_link3basedesy)**2)
        print("c: ", c)
        l1new = l1
        l2new = l2
        cnew = c

        print("cosine argument: ", (l1new**2 + l2new**2 - cnew**2)/(2*l1new*l2new))

        gamma = np.arccos((l1new**2 + l2new**2 - cnew**2)/(2*l1new*l2new))
        alpha = np.arcsin(l1*np.sin(gamma)/c)
        beta = np.pi - gamma - alpha
        phi = np.arccos(p_world_link3basedesx/c)

        print("[gamma, alpha, beta, phi]: ", gamma, alpha, beta, phi)

        if isnan(gamma) or isnan(alpha) or isnan(phi):
             print("Geometry Not Feasible")
             feasible = False

        link1_des = beta - phi
        link2_des = -(np.pi - gamma)
        link3_des = alpha + phi - 0.04

        if finger == "left":
                link_des[1] = link1_des
                link_des[2] = link2_des
                link_des[3] = link3_des
        if finger == "right":
                # Both fingers have the same angle orientations.  
                # Therefore, to convert from left to right, we just need to negate the angles
                link_des[1] = -link1_des
                link_des[2] = -link2_des
                link_des[3] = -link3_des

        return link_des, feasible
