#coding=utf-8

import rospy
from predictor_assistance.srv import AssistancePredictor, AssistancePredictorResponse
from control_manip.msg import GoalArray as goalArrayMsg
from shared_control_autonomy.srv import InitPred, InitPredResponse

import Utils
import RobotState as rs
from RobotState import Action
import RobotAssistancePolicy
import PrintOnFile as pof

import copy
import numpy as np

class PredictorAssistance:
    """
    PredictorAssistance Class \n
    Args:
        name_srv: name of initPred service
    """
    def __init__(self, name_srv):
        self._goals = None
        self._user_twist = np.zeros(6)
        self._ee_pose = np.zeros((4, 4)) #zero matrix
        self._ca_twist = np.zeros(6)
        self._file = pof.PrintOnFile()
        self._print_on_file = False
        self._robot_state = None
        self._policy = None
        self._service_init = rospy.Service(name_srv, InitPred, self.handleInitPred)
        self._service = rospy.Service("predictor_assistance_srv", AssistancePredictor, self.handleAssistance)
    

    def handleInitPred(self, request):
        """
        Service to initialize predictor node \n
        Args: request \n
        Return: response
        """
        self._goals = Utils.getGoal(request.goals)
        self._ee_pose = Utils.pose_to_mat(request.ee_pose)        
        self._robot_state = rs.RobotState(self._ee_pose, 0,0,0)
        self._policy = RobotAssistancePolicy.RobotAssistancePolicy(self._goals, self._robot_state, self._print_on_file, self._file)
        
        print("Predictor node received request!")
        return InitPredResponse(True)




    def handleAssistance(self, request):
        """
        Service predictor assistance. It receivers twist user and actual EE pose and computes distribution probability and assisted twist. \n
        Args: request \n
        Return: response
        """
        #Set user twist, CA twist and actual EE pose
        self.setUserTwist(request.user_input)
        self.setEEPose(request.ee_pose)

        #Update Robot State pose
        #self._robot_state.updateState(self._ee_pose)
        
        #Upadate policy
        action_u = Action(self._user_twist)
        self._policy.update(action_u)
        result_action = self._policy.get_action()

        #Get assistance twist, probability distribution and index of goal with max prob
        assistance_twist = result_action.getTwist()
        assistance_twist_msg = Utils.arrayToTwistMsg(assistance_twist)
        prob_distribution = self._policy.getDistribution()
        self._policy.visualize_prob()
        index_max = self._policy.getIndexMax()

        return AssistancePredictorResponse(assistance_twist_msg, prob_distribution, index_max)


    def setUserTwist(self, user_twist):
        """
        Set user twist \n
        Args: 
            user_twist: Twist message
        """
        self._user_twist[0:3] = np.array([user_twist.linear.x, user_twist.linear.y, user_twist.linear.z])
        self._user_twist[3:6] = np.array([user_twist.angular.x, user_twist.angular.y, user_twist.angular.z])


    def setCATwist(self, ca_twist):
        """
        Set CA twist \n
        Args: 
            ca_twist: Twist message
        """
        self._ca_twist[0:3] = np.array([ca_twist.linear.x, ca_twist.linear.y, ca_twist.linear.z])
        self._ca_twist[3:6] = np.array([ca_twist.angular.x, ca_twist.angular.y, ca_twist.angular.z])


    def setEEPose(self, pose):
        """
        Set pose matrix of EE \n
        Args: 
            pose: pose message of EE
        """
        self._ee_pose = Utils.pose_to_mat(pose)