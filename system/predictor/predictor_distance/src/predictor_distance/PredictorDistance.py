#coding=utf-8

import rospy
from shared_control_autonomy.Goal import Goal
from shared_control_autonomy import Utils
from control_manip.msg import GoalArray as goalArrayMsg
from predictor_distance.srv import DistancePredictor, DistancePredictorResponse

from shared_control_autonomy.srv import InitPred, InitPredResponse 

import GoalPredictor as gp
import numpy as np
import copy

class PredictorDistance:
    """
    PredictorDistance Class \n
    Args:
        name_srv: name of initPred service
    """
    def __init__(self, name_srv):
        self._goals = None
        self._targets_position = None
        self._goal_pred = None
        self._service_init = rospy.Service(name_srv, InitPred, self.handleInitPred)
        self._service = rospy.Service("predictor_distance_srv", DistancePredictor, self.handlePredition)


    def handleInitPred(self, request):
        """
        Service to initialize predictor node \n
        Args:
            request: request of service
        Return: response
        """
        self._goals, targets_position = Utils.getGoal(request.goals)
        self._goal_pred = gp.GoalPredictor(self._goals)
        self._targets_position = np.zeros(len(self._goals))
        
        print("Predictor node received request!")
        return InitPredResponse(True)
        

    def handlePredition(self,request):
        """
        Service prediction, receives distances, compute the distribution probability and return distribution and index of goal with max probability \n
        Args:
            request: request of service
        Return: response composed by distribution and index
        """
        distances = list(request.distance)
        for i in range(len(distances)):
            if(distances[i] == 0):
                distribution = self._goal_pred.getDistribution()
                return DistancePredictorResponse(distribution, i)
            self._goals[i].updateDistance(distances[i])
        self._goal_pred.updateDistribution()
        self._goal_pred.printDistribution()
        distribution = self._goal_pred.getDistribution()
        index = self._goal_pred.getIndexMaxProb()
        return DistancePredictorResponse(distribution, index)
    

