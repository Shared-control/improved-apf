#coding=utf-8


import roslib; roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String


import rospy
from geometry_msgs.msg import Pose
from control_manip.srv import InitObj as initobjSrv
from control_manip.srv import Move as moveSrv
from control_manip.msg import Goal as goalMsg
from control_manip.msg import GoalArray as goalarrayMsg
from control_manip.msg import Objects as objectsMsg
from control_manip.msg import Status as statusMsg
from control_manip.msg import Command as commandMsg
from shared_control.msg import InitPredictor as initPredictorMsg

from shared_control.srv import InitPred  as initPredSrv 

from predictor_distance.srv import DistancePredictor as distancePredictorSrv
from predictor_assistance.srv import AssistancePredictor as assistancePredictorSrv
from grasp.srv import GraspRank as graspSrv
from myo.srv import ResetMyo as ResetMyoSrv

import copy
import math
import sys
import time
import numpy as np
import sympy as smp
from enum import IntEnum

import Clik
import Goal
import RobotKinematics as rk
import UserInput
import Utils
import PrintFile
import PotentialFunction as pf


class UserCommand(IntEnum):
    #UserCommand
    HOME = 0
    TWIST = 1
    PICK = 2
    PLACE = 3
    FINISH = 4

class SharedControl:
    """
    Shared Control class \n
    Args:
        potential_params: list of potential field parameters in this order [threshold distance, repulsive gain, attractive gain]
        config_params: list of CLIK parameters in this order [delta time, vmax, diag, goal_radius]
        teleop: True if teleoperation mode is used, False otherwise
        predictor_type: "distance" if Distance predictor is used, "max_ent_ioc" if Max EntIOC is used
        robot_type: name of the robot
        grasp: True if grasp mode is used, False otherwise
        user_input_type: name of user input control type
        index_test: index of the test
        DH_params: list of Denavit-Hartenberg parameters in this order [q,a,d,alpha]
        name_user_test: name of the user
        dynamic_system: type of system, True if system is dynamic (asyncro), False otherwise
        gripper_active: True if there is a gripper, False otherwise
        escape_active: True if escape points are active, False otherwise
    """
    def __init__(self, potential_params, config_params, teleop, predictor_type, 
                 robot_type, grasp, user_input_type, index_test, DH_params, name_user_test,
                 dynamic_system, gripper_active, escape_active):

        self._potential_params = potential_params
        self._config_params = config_params
        self._vmax = self._config_params[1]
        self._teleop = teleop
        self._predictor_type = predictor_type
        self._robot_type = robot_type
        self._grasp = grasp
        self._user_input_type = user_input_type
        self._index_test = index_test
        self._DH_params = DH_params
        self._goal_radius = config_params[3]
        self._indexEE = len(self._DH_params[0])
        self._gripper_active = gripper_active
        if(self._gripper_active):
            self._indexGripper = self._indexEE + 1
        
        self._end_system = False
        self._distance_type = "distance"
        self._dynamic_system = dynamic_system
        self._escape_active = escape_active
        self._LEN_TWIST_VECTOR = 6

        self._ZT = 1.20

        #Variables for dynamic system: only obstacles can change, not goals
        if(self._dynamic_system):
            self._dynamic_first_lecture = True
            self._dynamic_goal = None
            self._dynamic_joints = None
            self._dynamic_obstacles = None
            self._dynamic_escape_points = None
            self._joints_target_positions = None
            self._dynamic_goal_msg = None
            self._dynamic_sub_objs = None


        #User Input, PrintFile and kinematics
        self._userinput = UserInput.UserInput(self._robot_type, self._user_input_type)
        self._print_file = None
        if(self._teleop):
            self._print_file = PrintFile.PrintFile("teleop", self._index_test, name_user_test, self._user_input_type)
        else:
            self._print_file = PrintFile.PrintFile(self._predictor_type, self._index_test, name_user_test, self._user_input_type)
        self._rob_kin = rk.RobotKinematics(self._DH_params[0], self._DH_params[1], self._DH_params[2], self._DH_params[3], self._robot_type, self._gripper_active)

        #Services        
        self._service_obj = None
        self._service_move = None
        self._service_grasp = None
        self._service_init_pred_node = None
        self._service_predictor = None
        self._reset_myo_service = None

    
    def initConnection(self):
        """
        Initialize connection to services
        """
        if(self._dynamic_system):
            self._dynamic_sub_objs = rospy.Subscriber(self._robot_type + "/objects_msg", objectsMsg, self.callbackTableObjects)
        else:
            name_objects_srv = self._robot_type + "/objects_srv"
            rospy.wait_for_service(name_objects_srv)
            try:
                self._service_obj = rospy.ServiceProxy(name_objects_srv, initobjSrv, persistent=True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" %e)
                rospy.on_shutdown("Error service")
                print("Shutdown")
                sys.exit()

        name_move_srv = self._robot_type + "/move_srv"
        rospy.wait_for_service(name_move_srv)
        try:
            self._service_move = rospy.ServiceProxy(name_move_srv, moveSrv, persistent=True)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" %e)
            rospy.on_shutdown("Error service")
            print("Shutdown")
            sys.exit()

        if(self._grasp):
            name_grasp_srv = self._robot_type + "/grasp_srv"
            rospy.wait_for_service(name_grasp_srv, 60)
            try:
                self._service_grasp = rospy.ServiceProxy(name_grasp_srv, graspSrv, persistent=True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" %e)
                rospy.on_shutdown("Error service")
                print("Shutdown")
                sys.exit()

        if(self._user_input_type == "myo"):
            name_reset_srv = self._robot_type + "/reset_myo_srv"
            rospy.wait_for_service(name_reset_srv, 60)
            try:
                self._reset_myo_service = rospy.ServiceProxy(name_reset_srv, ResetMyoSrv, persistent=True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" %e)
                rospy.on_shutdown("Error service")
                print("Shutdown")
                sys.exit()


    def initPredictorConnection(self):
        """
        Initialize connection to predictor service
        """
        name_init_pred_srv = self._robot_type + "/init_prediction_srv"
        rospy.wait_for_service(name_init_pred_srv, 60)
        try:
            self._service_init_pred_node = rospy.ServiceProxy(name_init_pred_srv, initPredSrv, persistent=True)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" %e)
            rospy.on_shutdown("Error service")
            print("Shutdown")
            sys.exit()

        if(self._predictor_type == self._distance_type):
            name_predictor_srv = self._robot_type + "/predictor_distance_srv"
            rospy.wait_for_service(name_predictor_srv, 60)
            try:
                self._service_predictor = rospy.ServiceProxy(name_predictor_srv, distancePredictorSrv, persistent=True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" %e)
                rospy.on_shutdown("Error service")
                print("Shutdown")
                sys.exit()
        else:
            name_predictor_srv = self._robot_type + "/predictor_assistance_srv"
            rospy.wait_for_service(name_predictor_srv, 60)
            try:
                self._service_predictor = rospy.ServiceProxy(name_predictor_srv, assistancePredictorSrv, persistent=True)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" %e)
                rospy.on_shutdown("Error service")
                print("Shutdown")
                sys.exit()


    def start(self):
        """
        Start system
        """
        self.initConnection()

        while(not self._end_system):
            if(self._teleop):
                print("teleop")
                self.teleopExecute()
            else:
                if(not self._dynamic_system):
                    print("static system")
                    self.staticExecute()                
                else:
                    print("dynamic Async system")
                    self.asyncDynamicExecute()

        print("Finish: turn off the system!")
        rospy.signal_shutdown("Fine")

    
    def teleopExecute(self):
        """
        Teleoperation version of the system
        """
        if(not self.serviceResetMyo(True)):
            rospy.logerr("ERROR WITH MYO")
            rospy.on_shutdown("Error movement")
            sys.exit()


        response_obj = self.serviceObject(True)
        init_q = Utils.getInitJoints(response_obj.objects.joints) 
        actual_pose_matrix_ee = self._rob_kin.get_pose(init_q, self._indexEE)
        actual_position_ee = self._rob_kin.getPosition(init_q, self._indexEE)
        print("Actual position EE: " +str(actual_position_ee))
        
        actual_position_gripper = np.zeros(3)
        if(self._gripper_active):
            actual_position_gripper = self._rob_kin.getPosition(init_q, self._indexGripper)
            print("Actual position gripper: " +str(actual_position_gripper))

        if(self._index_test != 0):
            self._print_file.newFile(self._index_test)
        if(self._gripper_active):
            self._print_file.write_with_title(actual_position_gripper, "Start")
        else:
            self._print_file.write_with_title(actual_position_ee, "Start")
        self._print_file.end_block()

        #Goal array
        goal_list = list()
        targets_position = list()
        goal_msg = response_obj.objects.goals
        goal_list, targets_position = Utils.getGoal(goal_msg)
        print("Num goals: " +str(len(goal_list)))

        self._print_file.write("Goals list")
        for goal in goal_list:
            self._print_file.write_with_title(goal.getCenterPosition(), goal.getID())
        self._print_file.end_block()

        #array of distances from ee
        init_dist = []
        for i in range(len(targets_position)):
            if(self._gripper_active):
                #dist = Utils.computeDistance(actual_position_gripper, targets_position[i])
                dist = Utils.computeDistanceXY(actual_position_gripper, targets_position[i])
                init_dist.append(dist)
            else:
                #dist = Utils.computeDistance(actual_position_ee, targets_position[i])
                dist = Utils.computeDistanceXY(actual_position_gripper, targets_position[i])
                init_dist.append(dist)

        #Init predictor service connection
        self.initPredictorConnection()

        #Use InitPred srv to initialize predictor node
        ee_pose_msg = Utils.matrixToPoseMsg(actual_pose_matrix_ee)
        if(not self.serviceInitPred(goal_msg, ee_pose_msg)):
            rospy.logerr("Error init predictor node")
            rospy.on_shutdown("Error")
            return

        #Positive matrix for CLIK
        K = self._config_params[2] * np.identity(6)
        clik_o = Clik.Clik(K, self._config_params[0], self._rob_kin, init_q)
        
        it = 0 #number of iterations
        att_q = init_q #for while cicle->current vector q
        distance = float('inf') #Init distance

        #init distribution: all goals are same probability
        goal_distrib = []
        for i in range(len(goal_list)):
            prob = 1./len(goal_list)
            goal_distrib.append(prob)
            print("Index: " +str(i) + " associated to: " +str(goal_list[i].getID()))
        #Index of goal with the highest probability
        index_max = 0

        #Time to reach target: timer start after first user command
        start_system = None

        z_actual = None

        print("Ready to move to goal")
        while(not ((distance < self._goal_radius) and (z_actual < self._ZT))):
            twist_user_input = np.zeros(self._LEN_TWIST_VECTOR)
            
            while(np.linalg.norm(twist_user_input) == 0):                
                twist_user_input = self._userinput.getTwist()
            twist_user_input = Utils.setTwist(twist_user_input, self._vmax)
            print("Twist User: " +str(twist_user_input))

            #Start timer
            if(it == 0):
                start_system = time.time()

            #Prediction and assistance service to compute probability and assistance twist
            if(self._predictor_type != self._distance_type):
                #There is a user command
                if(np.linalg.norm(twist_user_input) != 0):
                    response_assist = self.serviceAssistancePredictor(twist_user_input, np.zeros(self._LEN_TWIST_VECTOR), actual_pose_matrix_ee)
                    goal_distrib = response_assist.distr_prob
                    index_max = response_assist.index_max
                    #twist_a = Utils.twistMsgToArray(response_assist.assisted_twist)
                    #velocity saturation
                    #twist_a = Utils.setTwist(twist_a, self._vmax) 
                    #print("Twist A: " +str(twist_a))


            #CLIK and Inverse Kinematics
            jacob_matr = self._rob_kin.evaluateJacobian(att_q, self._indexEE)
            new_q_clik, new_pose_ee_clik, new_final_twist = clik_o.computeCLIK(actual_position_ee, twist_user_input, jacob_matr)

            #Send next pose to controller_ur_node for movement
            if(not self.serviceMove(commandMsg.MOVE, Utils.matrixToPoseMsg(new_pose_ee_clik))):
                rospy.logerr("Error movement!")
                rospy.on_shutdown("Error movement")
                sys.exit()

            #Update params for loop: joints, position_ee and distance
            att_q = new_q_clik
            actual_pose_matrix_ee = new_pose_ee_clik
            actual_position_ee = new_pose_ee_clik[0:3,3]
            
            if(self._gripper_active):
                actual_position_gripper = self._rob_kin.getPosition(att_q, self._indexGripper)

            for i in range(len(init_dist)):
                if(self._gripper_active):
                    #dist = Utils.computeDistance(actual_position_gripper, targets_position[i])
                    dist = Utils.computeDistanceXY(actual_position_gripper, targets_position[i])
                    init_dist[i] = dist
                else:
                    #dist = Utils.computeDistance(actual_position_ee, targets_position[i])
                    dist = Utils.computeDistanceXY(actual_position_ee, targets_position[i])
                    init_dist[i] = dist

            #Prediction based on distance: service to compute probability
            if(self._predictor_type == self._distance_type):
                response_dist = self.serviceDistancePredictor(init_dist)
                goal_distrib = response_dist.distr_prob 
                index_max = response_dist.index_max
            
            distance = init_dist[index_max]

            if(self._gripper_active):
                z_actual = actual_position_gripper[2]
            else:
                z_actual = actual_position_ee[2]
            
            #Print on file
            self._print_file.write_with_title(twist_user_input[0:3], "user_twist")
            if(np.linalg.norm(new_final_twist) != 0):
                self._print_file.write_with_title(new_final_twist[0:3], "final_twist")
            if(self._gripper_active):
                self._print_file.write_with_title(actual_position_gripper[0:3], "position_gripper")
            else:
                self._print_file.write_with_title(actual_position_ee[0:3], "position_ee")
            self._print_file.write_with_title(it, "iteration")
            self._print_file.write_with_title(goal_distrib, "distribution")
            self._print_file.end_block()

            print("Actual position: " +str(actual_position_ee))
            print("Gripper position: " +str(actual_position_gripper))
            #print("List distance: " +str(init_dist))
            print("Distance: " +str(distance))
            print("IT: " +str(it))
            it += 1
            #rospy.sleep(0.1)
        #END WHILE
        
        #Time
        end_system = time.time()
        interval = end_system - start_system
        print("Time: " + str(interval))
        print("Number of iteration: " +str(it))
        print("Distance final to goal: " +str(distance))

        self._print_file.write_with_title(interval, "time")
        self._print_file.end_block()

        #Grasp, pick and place
        goal_obj_selected = goal_list[index_max]
        print("Goal ID: " +str(goal_obj_selected.getID()))

        self._print_file.write_with_title(goal_obj_selected.getID(), "ID goal")
        self._print_file.close()
        self._index_test += 1

        if(self._grasp):
            self.pickPlaceRoutine(goal_obj_selected, actual_pose_matrix_ee)
        else:
            self.returnOrFinish()


    def staticExecute(self):
        """
        Static version of the execution of the routine for picking up the object with collision avoidance and predictor nodes
        """
        if(not self.serviceResetMyo(True)):
            rospy.logerr("ERROR WITH MYO")
            rospy.on_shutdown("Error movement")
            sys.exit()

        response_obj = self.serviceObject(True)
        init_q = Utils.getInitJoints(response_obj.objects.joints) 
        actual_pose_matrix_ee = self._rob_kin.get_pose(init_q, self._indexEE)
        actual_position_ee = self._rob_kin.getPosition(init_q, self._indexEE)
        print("Actual position EE: " +str(actual_position_ee))
        
        actual_position_gripper = np.zeros(3)
        if(self._gripper_active):
            actual_position_gripper = self._rob_kin.getPosition(init_q, self._indexGripper)
            print("Actual position gripper: " +str(actual_position_gripper))

        if(self._index_test != 0):
            self._print_file.newFile(self._index_test)
        if(self._gripper_active):
            self._print_file.write_with_title(actual_position_gripper, "Start")
        else:
            self._print_file.write_with_title(actual_position_ee, "Start")
        self._print_file.end_block()

        #Fixed Obstacles array
        obs_position = Utils.getListPoints(response_obj.objects.obstacles)

        #Escape points
        escape_points = Utils.getListPoints(response_obj.objects.escape_points)
        
        #Goal array
        goal_list = list()
        targets_position = list()
        goal_msg = response_obj.objects.goals
        goal_list, targets_position = Utils.getGoal(goal_msg)

        self._print_file.write("Goals list")
        for goal in goal_list:
            self._print_file.write_with_title(goal.getCenterPosition(), goal.getID())
        self._print_file.end_block()

        #array of distances from ee
        init_dist = []
        for i in range(len(targets_position)):
            if(self._gripper_active):
                #dist = Utils.computeDistance(actual_position_gripper, targets_position[i])
                dist = Utils.computeDistanceXY(actual_position_gripper, targets_position[i])
                init_dist.append(dist)
            else:
                #dist = Utils.computeDistance(actual_position_ee, targets_position[i])
                dist = Utils.computeDistanceXY(actual_position_gripper, targets_position[i])
                init_dist.append(dist)
                        
        #Init predictor service connection
        self.initPredictorConnection()

        #Use InitPred srv to initialize predictor node
        ee_pose_msg = Utils.matrixToPoseMsg(actual_pose_matrix_ee)
        if(not self.serviceInitPred(goal_msg, ee_pose_msg)):
            rospy.logerr("Error init predictor node")
            rospy.on_shutdown("Error")
            return

        #Init other params
        #Positive matrix for CLIK
        K = self._config_params[2] * np.identity(6)
        clik_o = Clik.Clik(K, self._config_params[0], self._rob_kin, init_q)
        pot_func = pf.PotentialFunction(self._potential_params[0],self._potential_params[1],self._potential_params[2], self._potential_params[3], 
                                        obs_position, self._rob_kin, init_q, targets_position, escape_points, self._print_file, self._gripper_active, self._escape_active)

        it = 0 #number of iterations
        att_q = init_q #for while cicle->current vector q
        distance = float('inf')
        
        #init distribution: all goals are same probability
        goal_distrib = []
        for i in range(len(goal_list)):
            prob = 1./len(goal_list)
            goal_distrib.append(prob)
            print("Index: " +str(i) + " associated to: " +str(goal_list[i].getID()))
        #Index of goal with the highest probability
        index_max = 0

        #Time to reach target: timer start after first user command
        start_system = None

        z_actual = None

        print("Ready to move to goal")
        while(not ((distance < self._goal_radius) and (z_actual < self._ZT))):
            #Final twist for ee
            final_twist = np.zeros(self._LEN_TWIST_VECTOR)
            #Twist from target prediction and assistance node
            twist_a = np.zeros(self._LEN_TWIST_VECTOR)
            #Twist from collision avoidance
            twist_ca = np.zeros(self._LEN_TWIST_VECTOR)
            #User input twist
            twist_user_input = np.zeros(self._LEN_TWIST_VECTOR)

            #Always wait user twist command
            while(np.linalg.norm(twist_user_input) == 0):                
                twist_user_input = self._userinput.getTwist()
            twist_user_input = Utils.setTwist(twist_user_input, self._vmax)
            print("Twist User: " +str(twist_user_input))

            #Start timer
            if(it == 0):
                start_system = time.time()

            #Prediction and assistance service to compute probability and assistance twist
            if(self._predictor_type != self._distance_type):
                #There is a user command
                if(np.linalg.norm(twist_user_input) != 0):
                    #Service to target assistance
                    response_assist = self.serviceAssistancePredictor(twist_user_input, twist_ca, actual_pose_matrix_ee)
                    goal_distrib = response_assist.distr_prob
                    index_max = response_assist.index_max
                    #twist_a = Utils.twistMsgToArray(response_assist.assisted_twist)
                    #velocity saturation
                    #twist_a = Utils.setTwist(twist_a, self._vmax) 
                    #print("Twist A: " +str(twist_a))

            #Compute twist_ca from potential field
            twist_ca = pot_func.getCATwist(att_q, self._vmax, goal_distrib)
            print("Twist CA: " +str(twist_ca))

            #Final twist
            final_twist = twist_ca + twist_user_input            
            print("Twist Final: " +str(final_twist))

            #CLIK and Inverse Kinematics
            jacob_matr = self._rob_kin.evaluateJacobian(att_q, self._indexEE)
            #Since CLIK is used to move, it is correct use EE although gripper is active
            new_q_clik, new_pose_ee_clik, new_final_twist = clik_o.computeCLIK(actual_position_ee, final_twist, jacob_matr)

            #Send next pose to controller_ur_node for movement
            if(not self.serviceMove(commandMsg.MOVE, Utils.matrixToPoseMsg(new_pose_ee_clik))):
                rospy.logerr("Error")
                rospy.on_shutdown("Error")
                return
            
            #Update params for loop: joints, position_ee (and gripper), distribution and distance
            att_q = new_q_clik
            actual_pose_matrix_ee = new_pose_ee_clik
            actual_position_ee = new_pose_ee_clik[0:3,3]

            if(self._gripper_active):
                actual_position_gripper = self._rob_kin.getPosition(att_q, self._indexGripper)

            for i in range(len(init_dist)):
                if(self._gripper_active):
                    #dist = Utils.computeDistance(actual_position_gripper, targets_position[i])
                    dist = Utils.computeDistanceXY(actual_position_gripper, targets_position[i])
                    init_dist[i] = dist
                else:
                    #dist = Utils.computeDistance(actual_position_ee, targets_position[i])
                    dist = Utils.computeDistanceXY(actual_position_ee, targets_position[i])
                    init_dist[i] = dist

            #Prediction based on distance: service to compute probability
            if(self._predictor_type == self._distance_type):
                response_dist = self.serviceDistancePredictor(init_dist)
                goal_distrib = response_dist.distr_prob 
                index_max = response_dist.index_max
            
            distance = init_dist[index_max]
            
            if(self._gripper_active):
                z_actual = actual_position_gripper[2]
            else:
                z_actual = actual_position_ee[2]

 
            #Print on file
            self._print_file.write_with_title(twist_user_input[0:3], "user_twist")
            self._print_file.write_with_title(twist_ca[0:3], "avoidance_twist")
            #self._print_file.write_with_title(twist_a[0:3], "assistance_twist")
            if(np.linalg.norm(new_final_twist) == 0):
                self._print_file.write_with_title(final_twist[0:3], "final_twist")
            else:
                self._print_file.write_with_title(new_final_twist[0:3], "final_twist")
            if(self._gripper_active):
                self._print_file.write_with_title(actual_position_gripper[0:3], "position_gripper")
            else:
                self._print_file.write_with_title(actual_position_ee[0:3], "position_ee")
            self._print_file.write_with_title(it, "iteration")
            self._print_file.write_with_title(goal_distrib, "distribution")
            self._print_file.end_block()

            print("Actual position: " +str(actual_position_ee))
            print("Gripper position: " +str(actual_position_gripper))
            #print("List distance: " +str(init_dist))
            print("Distance: " +str(distance))
            print("IT: " +str(it))
            it += 1

            #rospy.sleep(0.1)
        #END WHILE

        #Time
        end_system = time.time()
        interval = end_system - start_system
        print("Time: " + str(interval))
        print("Number of iteration: " +str(it))
        print("Distance final to goal: " +str(distance))

        self._print_file.write_with_title(interval, "time")
        self._print_file.end_block()
        
        #Grasp, pick and place
        goal_obj = goal_list[index_max]
        print("Goal ID: " +str(goal_obj.getID()))

        self._print_file.write_with_title(goal_obj.getID(), "ID goal")
        self._print_file.close()
        self._index_test += 1

        if(self._grasp):
            self.pickPlaceRoutine(goal_obj, actual_pose_matrix_ee)
        else:
            self.returnOrFinish()


    def returnOrFinish(self):
        """
        Manage return to HOME position or turn off the system.
        """
        
        if(not self.serviceMove(commandMsg.HOME)):
            rospy.logerr("Error when user sends the HOME command")
            rospy.on_shutdown("Error")
        

        '''
        if(self._user_input_type == "myo"):
            if(not self.serviceMove(commandMsg.HOME)):
                rospy.logerr("Error when user sends the HOME command")
                rospy.on_shutdown("Error")
        else:
            print("Insert HOME or FINISH command")
            while((self._userinput.getCommand() != UserCommand.FINISH) and (self._userinput.getCommand() != UserCommand.HOME)):
                rospy.logwarn("No Home or FINISH command: please insert HOME or FINISH command!")
                rospy.sleep(1.0)
            if(self._userinput.getCommand() == UserCommand.FINISH):
                self._end_system = True
                if(not self.serviceMove(commandMsg.FINISH)):
                    rospy.logerr("Error when user sends the FINISH command")
                    rospy.on_shutdown("Error")

            else:
                if(not self.serviceMove(commandMsg.HOME)):
                    rospy.logerr("Error when user sends the HOME command")
                    rospy.on_shutdown("Error")
        '''

    
    def pickPlaceRoutine(self, goal_obj, pose_matrix_ee):
        """
        Pick and Place routine \n
        Args:
            goal_obj: Goal object
            pose_matrix_ee: ee pose matrix
        """
        '''
        while(self._userinput.getCommand() != UserCommand.PICK):
            rospy.logwarn("No PICK command: please insert PICK command!")
            rospy.sleep(1.0)
        '''

        response_grasp = self.serviceGrasp(goal_obj, pose_matrix_ee)
        pick_done = False
        #Use for test
        pick_pose = Pose()
        for grasp_point in response_grasp.rank:
            if(self.serviceMove(commandMsg.PICK, grasp_point.pose)):
                pick_pose = grasp_point.pose
                print("Pick done!")
                rospy.sleep(0.5)
                pick_done = True
                break
            else:
                rospy.logwarn("Non valid grasp point")
        #Place
        if(pick_done):
            place_pose = Pose()
            place_pose = pick_pose           
            if(not self.serviceMove(commandMsg.PLACE, place_pose)):
                rospy.logerr("Error PLACE")
                rospy.on_shutdown("Error")
                return
            print("Place done!")
        
        self.returnOrFinish()
        

    def serviceResetMyo(self, reset=True):
        """
        Service to reset Myo pose \n
        Args:
            reset
        Return: response
        """
        if(self._user_input_type == "myo"):
            response = self._reset_myo_service.call(reset)
            return response
        return True


    def serviceInitPred(self, goals_vector_msg, ee_pose=Pose()):
        """
        Service to initialize predictor node \n
        Args:
            goals_vector_msg: Goal msg vector
            ee_pose: ee Pose msg
        Return: response
        """
        response = self._service_init_pred_node.call(goals_vector_msg, ee_pose)
        return response
        

    def serviceObject(self, status):
        """
        Service to received goals, obstacles, fixed escape points and initial value of joints
        Args:
            status: status
        Return: response
        """
        status_msg = statusMsg()
        status_msg.ready = status
        response = self._service_obj.call(status_msg)
        print("Received info")
        return response 


    def serviceAssistancePredictor(self, user_twist, potential_twist, ee_matrix):
        """
        Service to target assistance node
        Args:
            user_twist: user twist
            potential_twist: collision avoidance twist
            ee_matrix: pose ee
        Return: response
        """
        twist_u = Utils.arrayToTwistMsg(user_twist)
        twist_pot = Utils.arrayToTwistMsg(potential_twist)
        matrix_ee = Utils.matrixToPoseMsg(ee_matrix)
        response = self._service_predictor.call(twist_u, twist_pot, matrix_ee)
        return response 


    def serviceDistancePredictor(self, distances):
        """
        Service to distance predictor node \n
        Args:
            distances: distances from ee to goals
        Return: response
        """
        response = self._service_predictor.call(distances)
        return response


    def serviceGrasp(self, goal, ee_pose):
        """
        Service to compute rank of grasping points \n
        Args:
            goal: Goal object
            ee_pose: ee_pose matrix
        Return: grasping points rank
        """
        goal_msg = goalMsg()
        goal_msg.id = goal.getID()
        goal_msg.center = Utils.matrixToPoseMsg(goal.getCenterMatrix())
        goal_msg.grasping_points = goal.getGraspingPoints()

        pose_ee_msg = Utils.matrixToPoseMsg(ee_pose)
        response = self._service_grasp.call(goal_msg, pose_ee_msg)
        return response


    def serviceMove(self, type_command, pose=None):
        """
        Service to send movement to controller manipulator node \n
        Args:
            type_command: type of command
            pose: pose to go if type command is MOVE, PICK or PLACE
        Return: true if motion has been completed, false otherwise
        """
        command_msg = commandMsg()
        command_msg.command = type_command
        if(pose is not None):
            command_msg.to_pose = pose
        response = self._service_move.call(command_msg)
        return response


    def callbackTableObjects(self, data):
        """
        Callback to manage objects on the table
        """
        if(self._dynamic_first_lecture):
            self._dynamic_joints = Utils.getInitJoints(data.joints)
            self._dynamic_goal, self._joints_target_positions = Utils.getGoal(data.goals)
            self._dynamic_goal_msg = data.goals
            self._dynamic_first_lecture = False
        
        self._dynamic_obstacles = Utils.getListPoints(data.obstacles)
        self._dynamic_escape_points = Utils.getListPoints(data.escape_points)


    def asyncDynamicExecute(self):
        """
        Asyncro version of the execution of the routine for picking up the object with collision avoidance and predictor nodes
        """
        #Use to syncronize only the start of the system
        if(not self.serviceResetMyo(True)):
            rospy.logerr("ERROR WITH MYO")
            rospy.on_shutdown("Error movement")
            sys.exit()


        self._dynamic_first_lecture = True
        while(self._dynamic_first_lecture):
            rospy.sleep(0.01)

        actual_pose_matrix_ee = self._rob_kin.get_pose(copy.copy(self._dynamic_joints), self._indexEE)
        actual_position_ee = self._rob_kin.getPosition(copy.copy(self._dynamic_joints), self._indexEE)
        print("Actual position EE: " +str(actual_position_ee))
        
        actual_position_gripper = np.zeros(3)
        if(self._gripper_active):
            actual_position_gripper = self._rob_kin.getPosition(copy.copy(self._dynamic_joints), self._indexGripper)
            print("Actual position gripper: " +str(actual_position_gripper))

        if(self._index_test != 0):
            self._print_file.newFile(self._index_test)
        if(self._gripper_active):
            self._print_file.write_with_title(actual_position_gripper, "Start")
        else:
            self._print_file.write_with_title(actual_position_ee, "Start")
        self._print_file.end_block()

        #Goal array
        goal_list = list()
        targets_position = list()
        copy_goal_msg = copy.copy(self._dynamic_goal_msg)
        goal_list, targets_position = Utils.getGoal(copy_goal_msg)

        self._print_file.write("Goals list")
        for goal in goal_list:
            self._print_file.write_with_title(goal.getCenterPosition(), goal.getID())
        self._print_file.end_block()

        #array of distances from ee
        init_dist = []
        for i in range(len(targets_position)):
            if(self._gripper_active):
                dist = Utils.computeDistance(actual_position_gripper, targets_position[i])
                init_dist.append(dist)
            else:
                dist = Utils.computeDistance(actual_position_ee, targets_position[i])
                init_dist.append(dist)
        
        #Init predictor service connection
        self.initPredictorConnection()

        #Use InitPred srv to initialize predictor node
        ee_pose_msg = Utils.matrixToPoseMsg(actual_pose_matrix_ee)
        if(not self.serviceInitPred(copy_goal_msg, ee_pose_msg)):
            rospy.logerr("Error init predictor node")
            rospy.on_shutdown("Error")
            return

        #Init other params
        #Positive matrix for CLIK
        K = self._config_params[2] * np.identity(6)
        clik_o = Clik.Clik(K, self._config_params[0], self._rob_kin, self._dynamic_joints)
        pot_func = pf.PotentialFunction(self._potential_params[0],self._potential_params[1],self._potential_params[2], self._potential_params[3], 
                                        self._dynamic_obstacles, self._rob_kin, self._dynamic_joints, targets_position, self._dynamic_escape_points, 
                                        self._print_file, self._gripper_active, self._escape_active)
        
        it = 0 #number of iterations
        att_q = copy.copy(self._dynamic_joints) #for while cicle->current vector q
        distance = float('inf')

        #init distribution: all goals are same probability
        goal_distrib = []
        for i in range(len(goal_list)):
            prob = 1./len(goal_list)
            goal_distrib.append(prob)
            print("Index: " +str(i) + " associated to: " +str(goal_list[i].getID()))
        #Index of goal with the highest probability
        index_max = 0

        #Time to reach target: timer start after first user command
        start_system = None

        print("Ready to move to goal")
        while(distance > self._goal_radius):
            #Final twist for ee
            final_twist = np.zeros(self._LEN_TWIST_VECTOR)
            #Twist from target prediction and assistance node
            twist_a = np.zeros(self._LEN_TWIST_VECTOR)
            #Twist from collision avoidance
            twist_ca = np.zeros(self._LEN_TWIST_VECTOR)
            #User input twist
            twist_user_input = np.zeros(self._LEN_TWIST_VECTOR)

            #Always wait user twist command
            while(np.linalg.norm(twist_user_input) == 0):                        
                twist_user_input = self._userinput.getTwist()
            twist_user_input = Utils.setTwist(twist_user_input, self._vmax)
            print("Twist User: " +str(twist_user_input))

            #Start timer
            if(it == 0):
                start_system = time.time()

            #Prediction and assistance service to compute probability and assistance twist
            if(self._predictor_type != self._distance_type):
                #There is a user command
                if(np.linalg.norm(twist_user_input) != 0):
                    #Service to target assistance
                    response_assist = self.serviceAssistancePredictor(twist_user_input, twist_ca, actual_pose_matrix_ee)
                    goal_distrib = response_assist.distr_prob
                    index_max = response_assist.index_max
                    #twist_a = Utils.twistMsgToArray(response_assist.assisted_twist)
                    #velocity saturation
                    #twist_a = Utils.setTwist(twist_a, self._vmax) 
                    #print("Twist A: " +str(twist_a))

            #Compute twist_ca from potential field
            twist_ca = pot_func.getDynamicCATwist(att_q, self._vmax, goal_distrib, self._dynamic_obstacles, self._dynamic_escape_points)
            print("Twist CA: " +str(twist_ca))

            #Final twist
            final_twist = twist_ca + twist_user_input
            print("Twist Final: " +str(final_twist))

            #CLIK and Inverse Kinematics
            jacob_matr = self._rob_kin.evaluateJacobian(att_q, self._indexEE)
            new_q_clik, new_pose_ee_clik, new_final_twist = clik_o.computeCLIK(actual_position_ee, final_twist, jacob_matr)

            #Send next pose to controller_ur_node for movement
            if(not self.serviceMove(commandMsg.MOVE, Utils.matrixToPoseMsg(new_pose_ee_clik))):
                rospy.logerr("Error")
                rospy.on_shutdown("Error")
                return
            
            #Update params for loop: joints, position_ee, distribution and distance
            att_q = new_q_clik
            actual_pose_matrix_ee = new_pose_ee_clik
            actual_position_ee = new_pose_ee_clik[0:3,3]

            if(self._gripper_active):
                actual_position_gripper = self._rob_kin.getPosition(att_q, self._indexGripper)

            for i in range(len(init_dist)):
                if(self._gripper_active):
                    dist = Utils.computeDistance(actual_position_gripper, targets_position[i])
                    init_dist[i] = dist
                else:
                    dist = Utils.computeDistance(actual_position_ee, targets_position[i])
                    init_dist[i] = dist

            #Prediction based on distance: service to compute probability
            if(self._predictor_type == self._distance_type):
                response_dist = self.serviceDistancePredictor(init_dist)
                goal_distrib = response_dist.distr_prob 
                index_max = response_dist.index_max
            
            distance = init_dist[index_max]

            #Print on file
            self._print_file.write_with_title(twist_user_input[0:3], "user_twist")
            self._print_file.write_with_title(twist_ca[0:3], "avoidance_twist")
            self._print_file.write_with_title(twist_a[0:3], "assistance_twist")
            if(np.linalg.norm(new_final_twist) == 0):
                self._print_file.write_with_title(final_twist[0:3], "final_twist")
            else:
                self._print_file.write_with_title(new_final_twist[0:3], "final_twist")
            if(self._gripper_active):
                self._print_file.write_with_title(actual_position_gripper[0:3], "position_gripper")
            else:
                self._print_file.write_with_title(actual_position_ee[0:3], "position_ee")
            self._print_file.write_with_title(it, "iteration")
            self._print_file.write_with_title(goal_distrib, "distribution")
            self._print_file.end_block()

            print("Actual position: " +str(actual_position_ee))
            print("Gripper position: " +str(actual_position_gripper))
            #print("List distance: " +str(init_dist))
            print("Distance: " +str(distance))
            print("IT: " +str(it))
            it += 1
            #rospy.sleep(0.1)
        #END WHILE

        #Time
        end_system = time.time()
        interval = end_system - start_system
        print("Time: " + str(interval))
        print("Number of iteration: " +str(it))
        print("Distance final to goal: " +str(distance))

        self._print_file.write_with_title(interval, "time")
        self._print_file.end_block()
        
        #Grasp, pick and place
        goal_obj = goal_list[index_max]
        print("Goal ID: " +str(goal_obj.getID()))

        self._print_file.write_with_title(goal_obj.getID(), "ID goal")
        self._print_file.close()
        self._index_test += 1

        if(self._grasp):
            self.pickPlaceRoutine(goal_obj, actual_pose_matrix_ee)
        else:
            self.returnOrFinish()

