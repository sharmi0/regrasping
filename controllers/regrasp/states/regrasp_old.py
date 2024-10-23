# imports
import numpy as np
from controllers.base.baseState import *
from .fsm_config import *

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

        # Solve p_L + n_L*t = p_R + n_R*t for t
        t = (p_world_contactR - p_world_contactL) / (left_normal - right_normal)

        p_world_newCenter = p_world_contactL + t * left_normal

        print("p_world_newCenter: ",p_world_newCenter)
        

        new_x = p_world_newCenter[0] 
        current_x = (p_world_contactL[0] + p_world_contactR[0]) / 2

        new_yL = p_world_contactL[1]
        new_yR = p_world_contactR[1]


        print("Current q: ", GP.gr_data.get_q(GP.gr_data.all_idxs))

    # def calculate_coordinates(self, finger, x_des, y_des, tip_angle):
        new_x -= fsm_params.l3  # subtract the distance from the fingertip to the end of the finger

        # Get clearance coordinates
        new_yLClearance = new_yL + 0.01
        new_yRClearance = new_yR - 0.01

        valid, left_q_des_clearance = self.calculate_coordinates("left", new_x, new_yLClearance, 0.0)
        if valid:
            self.new_q_des_pregrasp[1:5] = left_q_des_clearance
        valid, right_q_des_clearance = self.calculate_coordinates("right", new_x, new_yRClearance, 0.0)
        if valid:
            self.new_q_des_pregrasp[5:9] = right_q_des_clearance    


        # Get grasp coordinates
        new_yL_grasp = new_yL - 0.01
        new_yR_grasp = new_yR + 0.01
        valid, self.left_q_des = self.calculate_coordinates("left", new_x, new_yL_grasp, 0.0)
        if valid:
            self.new_q_des_grasp[1:5] = self.left_q_des
        valid, self.right_q_des = self.calculate_coordinates("right", new_x, new_yR_grasp, 0.0)
        if valid:
            self.new_q_des_grasp[5:9] = self.right_q_des




# from old FSM:
#     def enter(self, MP):
#         print(self.name)

#         # get current arm vals, current finger vals
#         self.current_q = MP.q_arm.copy()
#         self.current_q_left = MP.q_left_finger.copy()
#         self.current_q_right = MP.q_right_finger.copy()
#         self.regrasp_type = "FAIL"

#         # calculate sphere estimate here based on previous contact (in gripper frame)
#         nc_1 = MP.T_base_lcont.R[:,2]
#         pc_1 = MP.T_base_lcont.t
#         nc_2 = MP.T_base_rcont.R[:,2]
#         pc_2 = MP.T_base_rcont.t
#         points = [pc_1,pc_2]
#         normals = [nc_1,nc_2]
#         r_center, p_base_center = self.sphere_estimate(points,normals)
#         print("Object is approx. " + str(r_center*2.0) + "m in diameter at [x,y,z]=" + str(p_base_center) + " in the gripper frame.")
#         dxl = MP.p_left_finger[0]-p_base_center[0]
#         dxr = MP.p_right_finger[0]-p_base_center[0]
#         print("Left fingertip is " + str(dxl) + "m from object center in x-direction.")
#         print("Right fingertip is " + str(dxr) + "m from object center in x-direction.")

#         # TODO: check finger angle signs here!!!!!

#         # TODO: testing drag decision logic, will add state for this next
#         if ((dxl<0) and (dxr<0)):
#             print("Starting drag based on fingertip locations.")
#             self.timer = time.time()
#             self.start_p2l = MP.p2_left_finger.copy()
#             self.start_p2r = MP.p2_right_finger.copy()
#             self.start_ql = MP.q_left_finger.copy()
#             self.start_qr = MP.q_right_finger.copy()
#             palm_start_drag = MP.d_palm.copy()
#             self.drag_time = time.time()
#             self.drag_traj_time = 1.0
#             drag_distance = -(palm_start_drag-0.060)
#             print("Drag distance: " + str(drag_distance))
#             left_kin, self.left_joint_des = self.calculate_coordinates("left", MP.p2_left_finger, drag_distance, 0.0, MP.p_left_finger[2,0], MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#             right_kin, self.right_joint_des = self.calculate_coordinates("right", MP.p2_right_finger, drag_distance, 0.0, MP.p_right_finger[2,0], MP)
#             if (left_kin and right_kin):
#                 print("Drag Kinematics Works, Begin Drag")
#                 self.drag_state = 1
#                 self.regrasp_type = "DRAG"
#             else:
#                 print("Move Kinematics Failed")
#                 # exit on failure
#                 self.regrasp_type = "FAIL"

#         elif((dxl>0) and (dxr>0)):
#             if (r_center>0.030):
#                 print("Should attempt a power grasp, based on fingertip locations and object size. ")
#                 self.start_ql = MP.q_left_finger.copy()
#                 self.start_qr = MP.q_right_finger.copy()
#                 move_xl = 0.02 # TODO: tune these distances
#                 move_xr = 0.02
#                 move_yl = 0.01
#                 move_yr = -0.01
#                 left_tip_angle = -(MP.q_left_finger[0]+MP.q_left_finger[1]+MP.q_left_finger[2]) #-(left_force_filt[4]*3.14/180)
#                 right_tip_angle = -(MP.q_right_finger[0]+MP.q_right_finger[1]+MP.q_right_finger[2]) #+(right_force_filt[4]*3.14/180)
#                 left_logic, self.left_joint_des = self.calculate_coordinates("left", MP.p2_left_finger, move_xl, move_yl, left_tip_angle, MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#                 right_logic, self.right_joint_des = self.calculate_coordinates("right", MP.p2_right_finger, move_xr, move_yr, right_tip_angle, MP)
#                 # TODO: case could be triggered by object being off-center and thus not seen by palm sensor, should we try to balance joint angles?
#                 print("Calculated joint angles:")
#                 print(self.left_joint_des)
#                 print(self.right_joint_des)
#                 # balanced_joints = 0.5*(np.array(np.abs(left_joint_des)) + np.array(np.abs(right_joint_des))) # take average joint angle magnitude
#                 # left_joint_des = np.multiply(np.sign(left_joint_des), balanced_joints) # apply correct signs
#                 # right_joint_des = np.multiply(np.sign(right_joint_des), balanced_joints) # apply correct signs
#                 # print("Balanced joint angles:")
#                 # print(left_joint_des)
#                 # print(right_joint_des)
#                 if (left_logic and right_logic):
#                     print("Move Kinematics Works, Begin Move")
#                     self.move_time= time.time()
#                     print(MP.d_palm)
#                     self.regrasp_type = "POWER"
#                 else:
#                     print("Move Kinematics Failed")
#                     # exit on failure
#                     self.regrasp_type = "FAIL"

#             else:
#                 print("Should attempt an antipodal grasp, based on fingertip locations and object size.")
#                 self.start_ql = MP.q_left_finger.copy()
#                 self.start_qr = MP.q_right_finger.copy()
#                 r_eps1 = 0.015 # extra clearance around radius of object
#                 r_eps2 = 0.005
#                 pl_cont = MP.T_base_lcont.t
#                 pr_cont = MP.T_base_rcont.t
#                 move_xl = p_base_center[0]-MP.p_left_finger[0]
#                 move_xr = p_base_center[0]-MP.p_right_finger[0]
#                 move_yl = p_base_center[1]+r_center[0]-MP.p_left_finger[1]
#                 move_yr = p_base_center[1]-r_center[0]-MP.p_right_finger[1]
#                 left_logic1, self.left_joint_des1 = self.calculate_coordinates("left", MP.p2_left_finger, move_xl, move_yl+r_eps1, 0.0, MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#                 right_logic1, self.right_joint_des1 = self.calculate_coordinates("right", MP.p2_right_finger, move_xr, move_yr-r_eps1, 0.0, MP)
#                 left_logic2, self.left_joint_des2 = self.calculate_coordinates("left", MP.p2_left_finger, move_xl, move_yl+r_eps2, 0.0, MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#                 right_logic2, self.right_joint_des2 = self.calculate_coordinates("right", MP.p2_right_finger, move_xr, move_yr-r_eps2, 0.0, MP)
#                 if (left_logic1 and right_logic1 and left_logic2 and right_logic2):
#                     print("Move Kinematics Works, Begin Move")
#                     self.move_time= time.time()
#                     print(MP.d_palm)
#                     self.regrasp_type = "ANTIPODAL"
#                 else:
#                     print("Move Kinematics Failed")
#                     # exit on failure
#                     self.regrasp_type = "FAIL"

#         else:
#             print("Fingertip locations are on opposite sides of the object center, attempting power regrasp.")
#             # TODO: debug this, how often does it enter this state? should it go to the antipodal state?
#             # if it should go to antipodal state, can get rid of this case and combine with above cases
#             # state = GRASP_FAIL
#             self.start_ql = MP.q_left_finger.copy()
#             self.start_qr = MP.q_right_finger.copy()
#             move_xl = 0.02 # TODO: tune these distances
#             move_xr = 0.02
#             move_yl = 0.01
#             move_yr = -0.01
#             left_tip_angle = -(MP.q_left_finger[0]+MP.q_left_finger[1]+MP.q_left_finger[2]) #-(left_force_filt[4]*3.14/180)
#             right_tip_angle = -(MP.q_right_finger[0]+MP.q_right_finger[1]+MP.q_right_finger[2]) #+(right_force_filt[4]*3.14/180)
#             left_logic, self.left_joint_des = self.calculate_coordinates("left", MP.p2_left_finger, move_xl, move_yl, left_tip_angle, MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#             right_logic, self.right_joint_des = self.calculate_coordinates("right", MP.p2_right_finger, move_xr, move_yr, right_tip_angle, MP)
#             # TODO: case could be triggered by object being off-center and thus not seen by palm sensor, should we try to balance joint angles?
#             print("Calculated joint angles:")
#             print(self.left_joint_des)
#             print(self.right_joint_des)
#             # balanced_joints = 0.5*(np.array(np.abs(left_joint_des)) + np.array(np.abs(right_joint_des))) # take average joint angle magnitude
#             # left_joint_des = np.multiply(np.sign(left_joint_des), balanced_joints) # apply correct signs
#             # right_joint_des = np.multiply(np.sign(right_joint_des), balanced_joints) # apply correct signs
#             # print("Balanced joint angles:")
#             # print(left_joint_des)
#             # print(right_joint_des)
#             if (left_logic and right_logic):
#                 print("Move Kinematics Works, Begin Move")
#                 self.move_time= time.time()
#                 print(MP.d_palm)
#                 self.regrasp_type = "POWER"
#             else:
#                 print("Move Kinematics Failed")
#                 # exit on failure
#                 self.regrasp_type = "FAIL"

#         self.enabled = 1



    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):

        # stay in this state
        next_state = self.name

        # get current time
        cur_time = GP.time()

        
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
            next_state = "LiftObject"

        # check for manual state transition to reset
        if GP.char_in=='R' or GP.char_in=='r':
            next_state = "Reset"


        return next_state

    def calculate_coordinates(self, finger, x_des, y_des, tip_angle):
        # x_des, y_des are the center of the tip joint
        print("x_des_raw: ", x_des)
        
        # x_des, y_des are in the gripper coordinate frame
        assert finger in ["left", "right"], "finger must be 'left' or 'right'"

        # Covert everything to the left finger frame
        if finger == "right":
            y_des = -y_des
        
        # Covert from gripper coordinate frame to finger coordinate frame
        x_des = x_des - fsm_params.t_gripper_fingerMidpoint[0]
        y_des = y_des - fsm_params.finger_spacing / 2

        print("x_des: ", x_des)
        print("y_des: ", y_des)
        
        # Solve closed form inverse kinematics
        link_des=np.zeros((4,))

        l1 = fsm_params.l1
        l2 = fsm_params.l2
        l3 = fsm_params.l3

        c= np.power((np.power(x_des,2) + np.power(y_des,2)),0.5) #the base of the triangle extending from base of l1 to tip of l2
        print(c, l1 + l2 )

        if c > l1 + l2 or l1 > l2 + c or l2 > l1 + c:
            print("Desired Triangle Not Geometrically Possible")
            return False, link_des
        
        angle_tri= np.arctan(y_des/x_des)
        angle_b= np.arccos((np.power(l1,2)+np.power(c,2)-np.power(l2,2))/(2*l1*c))  # b is acros from l2
        angle_c= np.arccos((np.power(l1,2)+np.power(l2,2)-np.power(c,2))/(2*l1*l2))  # c is across from c

        link1_des= angle_tri+angle_b
        link2_des= -np.pi+angle_c    
        link3_des= tip_angle - link1_des - link2_des

        if finger == "right":
            ...
            # TODO: Invert
        
        if finger == "left":
            link_des[1] = link1_des  # l1
            link_des[2] = link2_des  # l2
            link_des[3] = link3_des  # l3

        if finger == "right":
            # Both fingers have the same angle orientations.  
            # Therefore, to convert from left to right, we just need to negate the angles
            link_des[1] = -link1_des
            link_des[2] = -link2_des
            link_des[3] = -link3_des
        # link_des[0] = link1_des
        # link_des[1] = link2_des
        # link_des[2] = link3_des
        # link_des[3] = 0.0 # added for abad joint

        # print(link_des)
        # link_des[0] = 0.2, 

        return True, link_des
        




        if ((c< (MP.params.l1+MP.params.l2))and (MP.params.l1<(MP.params.l2+c)) and (MP.params.l2<(MP.params.l1+c))): #Check if desired point can form a triangle with length

            angle_tri= np.arctan(y_des/x_des)
            angle_b= np.arccos((np.power(MP.params.l1,2)+np.power(c,2)-np.power(MP.params.l2,2))/(2*MP.params.l1*c))
            angle_c= np.arccos((np.power(MP.params.l1,2)+np.power(MP.params.l2,2)-np.power(c,2))/(2*MP.params.l1*MP.params. l2))

            if (finger == "left"):
                link1_des= angle_tri+angle_b
                link2_des= -180*3.14/180+angle_c

            elif (finger == "right"):
                link1_des= angle_tri-angle_b #since angle_b will always be a positive number
                link2_des= 180*3.14/180-angle_c

            # link3_des= angles_start[0]+angles_start[1]+angles_start[2]-link1_des-link2_des
            link3_des = tip_angle - link1_des - link2_des

            # for new fingers, need to make each of these negative
            link_des[0]= -link1_des
            link_des[1]= -link2_des
            link_des[2]= -link3_des
            link_des[3]= 0.0 # added for abad joint

            return True, link_des

            # if (abs(link1_des-angles[0])<(5*3.14/180)): #if the desired angle of link 1 (tri + b) is less than x degrees from the current link1, things look right
            #     angle1_check=True
            # if (abs(link2_des-angles[1])<(5*3.14/180)):
            #     angle2_check=True

        else:
            print("Desired Triangle Not Geometrically Possible")
            return False, link_des


# from old FSM:
#     def execute(self, MP):

#         # stay in this state
#         next_state = self.name

#         # set arm commands
#         MP.q_arm_des[0:7] = self.current_q[0:7] # joint space desired angles
#         MP.qd_arm_des = np.zeros((7,))
#         MP.kp_arm = MP.params.kp_arm_default.copy()
#         MP.kd_arm = MP.params.kd_arm_default.copy()
#         # set grav comp torques
#         MP.tau_arm_des = MP.tau_arm_des_gc.copy()

#         # set fingers
#         MP.q_left_finger_des = self.current_q_left.copy()
#         MP.qd_left_finger_des = np.zeros((4,))
#         MP.tau_left_finger_des = np.zeros((4,))
#         MP.kp_left_finger = MP.params.Kp_jointsl.copy()
#         MP.kd_left_finger = MP.params.Kd_jointsl.copy()
#         MP.q_right_finger_des = self.current_q_right.copy()
#         MP.qd_right_finger_des = np.zeros((4,))
#         MP.tau_right_finger_des = np.zeros((4,))
#         MP.kp_right_finger = MP.params.Kp_jointsr.copy()
#         MP.kd_right_finger = MP.params.Kd_jointsr.copy()

#         if self.regrasp_type=="POWER":

#             move_traj_time = 0.25
#             close_fingers_time = 0.25
#             # move fingertips
#             if (MP.current_time - self.move_time < move_traj_time):
#                 ratio = (MP.current_time - self.move_time)/move_traj_time
#                 command_left = self.left_joint_des*ratio + self.start_ql*(1-ratio)
#                 command_right = self.right_joint_des*ratio + self.start_qr*(1-ratio)
#                 MP.q_left_finger_des = command_left.copy()
#                 MP.q_right_finger_des = command_right.copy()
#             # close with feed-forward torques
#             elif ((MP.current_time- self.move_time - move_traj_time) < close_fingers_time):
#                 MP.tau_left_finger_des = np.array([0.1, 0.1, 0.05, 0.0])
#                 MP.tau_right_finger_des = np.array([-0.1, -0.1, -0.05, 0.0])

#             # ready to grasp again
#             else:
#                 next_state = "ExecuteGrasp"

#         elif self.regrasp_type=="ANTIPODAL":

#             move_traj_time1 = 0.25
#             move_traj_time2 = 0.25 # time after end of first movement
#             close_fingers_time = 0.25 # time after end of second movement
#             # move fingertips
#             if (MP.current_time - self.move_time < move_traj_time1):
#                 ratio = (MP.current_time - self.move_time)/move_traj_time1
#                 command_left = self.left_joint_des1*ratio + self.start_ql*(1-ratio)
#                 command_right = self.right_joint_des1*ratio + self.start_qr*(1-ratio)
#                 MP.q_left_finger_des = command_left.copy()
#                 MP.q_right_finger_des = command_right.copy()
#             elif ((MP.current_time - self.move_time - move_traj_time1) < move_traj_time2):
#                 ratio = (MP.current_time - self.move_time - move_traj_time1)/move_traj_time2
#                 command_left = self.left_joint_des2*ratio + self.left_joint_des1*(1-ratio)
#                 command_right = self.right_joint_des2*ratio + self.right_joint_des1*(1-ratio)
#                 MP.q_left_finger_des = command_left.copy()
#                 MP.q_right_finger_des = command_right.copy()
#             # close along normal
#             elif ((MP.current_time - self.move_time - move_traj_time1 - move_traj_time2) < close_fingers_time):
#                 # TODO: this doesn't work in free space, since the normal changes
#                 close_force= 1.0 # TODO: tune this force
#                 Fl, taul = self.close_along_normal(close_force, MP.p_left_finger, MP.e_left_finger, MP.J_left_finger, MP.v_left_finger)
#                 Fr, taur = self.close_along_normal(close_force, MP.p_right_finger, MP.e_right_finger, MP.J_right_finger, MP.v_right_finger)
#                 MP.tau_left_finger_des[0:3] = taul[0:3,0]
#                 MP.kp_left_finger[0:3] = np.zeros((3,))
#                 MP.tau_right_finger_des[0:3] = taur[0:3,0]
#                 MP.kp_right_finger[0:3] = np.zeros((3,))

#             # ready to grasp again
#             else:
#                 next_state = "ExecuteGrasp"

#         elif self.regrasp_type=="DRAG":

#             pinch_force= 1.0 #tuned
#             # Fl, taul = pinch_along_normal(pinch_force,pl,el,Jl,vl)
#             # Fr, taur = pinch_along_normal(pinch_force,pr,er,Jr,vr)
#             Fl, taul = self.close_along_normal(pinch_force, MP.p_left_finger, MP.e_left_finger, MP.J_left_finger, MP.v_left_finger)
#             Fr, taur = self.close_along_normal(pinch_force, MP.p_right_finger, MP.e_right_finger, MP.J_right_finger, MP.v_right_finger)

#             # check for curled fingers
#             # TODO: test this transition and the finger joint limits in finger_kinematics.py
#             # TODO: is going all the way out to grasp-begin a good idea? maybe have an intermediate pose?
#             # if(check_curled_fingers(left_finger_q, right_finger_q)):
#             #     #print("Fingers are curled, resetting finger positions.")
#                 # set_left_finger(left_q_des_def, Kp_joints_def, Kd_joints_def, left_trq_des_def, left_abad_ilim_def)
#                 # set_right_finger(right_q_des_def, Kp_joints_def, Kd_joints_def, right_trq_des_def, right_abad_ilim_def)
#             #     #state = GRASP_BEGIN or EXECUTE GRASP? intermediate state like FINGER_RESET?


#             if (self.drag_state==1):
#                 #failures are usually here whenthe last joint slips which mean the ff torque curls it in so then the move has it culed in extra, need an intermediary to splay fingers out
#                 if (MP.current_time - self.drag_time < self.drag_traj_time):
#                     ratio = (MP.current_time - self.drag_time)/self.drag_traj_time
#                     command_left = self.left_joint_des*ratio + self.start_ql*(1-ratio)
#                     command_right = self.right_joint_des*ratio + self.start_qr*(1-ratio)
#                     MP.q_left_finger_des = command_left.copy()
#                     MP.q_right_finger_des = command_right.copy()
#                     MP.tau_left_finger_des[0:3] = taul[0:3,0]
#                     MP.tau_right_finger_des[0:3] = taur[0:3,0]
#                 else:
#                     self.drag_state=2
#                     print("Drag successful: "+ str(time.time()-self.timer) )
#                     self.wait_time= time.time()

#             elif (self.drag_state==2):
#                 # pause here
#                 drag_pause_time = 0.1
#                 MP.tau_left_finger_des[0:3] = taul[0:3,0]
#                 MP.kp_left_finger[0:3] = np.zeros((3,))
#                 MP.tau_right_finger_des[0:3] = taur[0:3,0]
#                 MP.kp_right_finger[0:3] = np.zeros((3,))
#                 if (time.time()-self.wait_time > drag_pause_time):
#                     self.drag_state=3

#             elif (self.drag_state==3):
#                 self.start_ql = MP.q_left_finger.copy()
#                 self.start_qr = MP.q_right_finger.copy()

#                 move_distance = 0.025 #.015 # increased to get more clearance post drag
#                 left_logic, self.left_joint_des = self.calculate_coordinates("left", MP.p2_left_finger, 0.0 , move_distance, 0.4, MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#                 right_logic, self.right_joint_des = self.calculate_coordinates("right", MP.p2_right_finger, 0.0, -move_distance, -0.4, MP)
#                 if (left_logic and right_logic):
#                     print("Move Kinematics Works, Begin Move")
#                     self.drag_state=4
#                     self.move_time= time.time()
#                 else:
#                     print("Move Kinematics Failed")
#                     # exit on failure
#                     next_state = "GoToHome"

#             elif (self.drag_state==4):
#                 move_traj_time = 0.25
#                 if (MP.current_time - self.move_time < move_traj_time):
#                     ratio = (MP.current_time - self.move_time)/move_traj_time
#                     command_left = self.left_joint_des*ratio + self.start_ql*(1-ratio)
#                     command_right = self.right_joint_des*ratio + self.start_qr*(1-ratio)
#                     MP.q_left_finger_des = command_left.copy()
#                     MP.q_right_finger_des = command_right.copy()
#                 else:
#                     self.drag_state=5

#             elif (self.drag_state==5):
#                 self.start_ql = MP.q_left_finger.copy()
#                 self.start_qr = MP.q_right_finger.copy()
#                 move_distance = .015 # TODO: increase this?
#                 left_logic, self.left_joint_des = self.calculate_coordinates("left", self.start_p2l, 0.0 , move_distance, 0.0, MP)
#                 right_logic, self.right_joint_des = self.calculate_coordinates("right", self.start_p2r, 0.0, -move_distance, 0.0, MP) # changed final argument to world tip angle (i.e. desired pl[2] or pr[2])
#                 if (left_logic and right_logic):
#                     print("Second Move Kinematics Works, Begin Move")
#                     self.drag_state=6
#                     self.move_time= time.time()
#                 else:
#                     print("Second Move Kinematics Failed")
#                     next_state = "GoToHome"

#             elif (self.drag_state==6):
#                 move_traj_time= 0.25
#                 if (MP.current_time - self.move_time < move_traj_time):
#                     ratio = (MP.current_time - self.move_time)/move_traj_time
#                     command_left = self.left_joint_des*ratio + self.start_ql*(1-ratio)
#                     command_right = self.right_joint_des*ratio + self.start_qr*(1-ratio)
#                     MP.q_left_finger_des = command_left.copy()
#                     MP.q_right_finger_des = command_right.copy()
#                 else:
#                     # ready to grasp again
#                     # TODO: if 0.25s close along normal works in other regrasps, could try here too?
#                     print("Finished drag regrasp.")
#                     next_state = "ExecuteGrasp"

#         else: # self.regrasp_type=="FAIL":
#             print("Fail.")
#             next_state = "GoToHome"


#         # check inputs for state transitions
#         if MP.char_in == '\x67': # press g
#             next_state = "GravityComp"
#         elif MP.char_in == '\x1B': # press esc
#             next_state = "GravityComp"

#         return next_state

#     ### helper functions ###

#     def sphere_estimate(self, points, normals):
#         # points is list of (3,) arrays
#         # normals is list of (3,) arrays
#         # returns radius and object center
#         # inputs and outputs are in the same frame

#         # solve for radius and center location of contact sphere
#         # nc_1 = T_w_ffcont.R[:,2]
#         # pc_1 = T_w_ffcont.t
#         # nc_2 = T_w_afcont.R[:,2]
#         # pc_2 = T_w_afcont.t

#         nc_1 = normals[0] # first (3,) array
#         nc_2 = normals[1] # second (3,) array

#         pc_1 = points[0] # first (3,) array
#         pc_2 = points[1] # second (3,) array

#         Ac = np.zeros((6,4))
#         Ac[0:3,0:3] = np.eye(3)
#         Ac[3:6,0:3] = np.eye(3)
#         Ac[0:3,3] = -nc_1
#         Ac[3:6,3] = -nc_2
#         bc = np.zeros((6,1))
#         bc[0:3,0] = pc_1
#         bc[3:6,0] = pc_2
#         xc = np.matmul( np.linalg.pinv(Ac), bc)
#         p_cent = xc[0:3,0]
#         r_cent = xc[3]
#         return r_cent, p_cent

#     # forward kinematics for fingers
#     # angles is list or 1D array of three finger joint angles (base, middele, tip)
#     # returns tip position, list of sensor normal vectors, and finger Jacobian
#     def calculate_coordinates(self, finger, p2, x_shift, y_shift, tip_angle, MP):
#         link_des=np.zeros((4,))
#         #p2[0]= x coordinate, p2[1]= y coordinate
#         x_des= p2[0]+x_shift
#         y_des= p2[1]+y_shift
#         c= np.power((np.power(x_des,2) + np.power(y_des,2)),0.5) #the base of the triangle extending from base of l1 to tip of l2

#         if ((c< (MP.params.l1+MP.params.l2))and (MP.params.l1<(MP.params.l2+c)) and (MP.params.l2<(MP.params.l1+c))): #Check if desired point can form a triangle with length

#             angle_tri= np.arctan(y_des/x_des)
#             angle_b= np.arccos((np.power(MP.params.l1,2)+np.power(c,2)-np.power(MP.params.l2,2))/(2*MP.params.l1*c))
#             angle_c= np.arccos((np.power(MP.params.l1,2)+np.power(MP.params.l2,2)-np.power(c,2))/(2*MP.params.l1*MP.params. l2))

#             if (finger == "left"):
#                 link1_des= angle_tri+angle_b
#                 link2_des= -180*3.14/180+angle_c

#             elif (finger == "right"):
#                 link1_des= angle_tri-angle_b #since angle_b will always be a positive number
#                 link2_des= 180*3.14/180-angle_c

#             # link3_des= angles_start[0]+angles_start[1]+angles_start[2]-link1_des-link2_des
#             link3_des = tip_angle - link1_des - link2_des

#             # for new fingers, need to make each of these negative
#             link_des[0]= -link1_des
#             link_des[1]= -link2_des
#             link_des[2]= -link3_des
#             link_des[3]= 0.0 # added for abad joint

#             return True, link_des

#             # if (abs(link1_des-angles[0])<(5*3.14/180)): #if the desired angle of link 1 (tri + b) is less than x degrees from the current link1, things look right
#             #     angle1_check=True
#             # if (abs(link2_des-angles[1])<(5*3.14/180)):
#             #     angle2_check=True

#         else:
#             print("Desired Triangle Not Geometrically Possible")
#             return False, link_des

#     # TODO: should helpers be in this class or in BaseState class?
#     def close_along_normal(self,force,p,e,J,v):
#         tau_ff = np.zeros((3,1))
#         F = np.zeros((2,1))
#         F = force*e[0:2,2:3] #ein
#         # convert forces to joint torques (for first two links only)
#         J2 = J[0:2,0:3]
#         tau_ff = np.matmul(J2.T, F)
#         return F, tau_ff

#     def pinch_along_normal(self,force,p,e,J,v): #about the same as the close_along_normal even with the 45 degree grasp case
#         tau_ff = np.zeros((3,1))
#         F = np.zeros((2,1))
#         F = force*e[0:2,3:4] #econ
#         # convert forces to joint torques (for first two links only)
#         J2 = J[0:2,0:3]
#         tau_ff = np.matmul(J2.T, F)
#         return F, tau_ff
