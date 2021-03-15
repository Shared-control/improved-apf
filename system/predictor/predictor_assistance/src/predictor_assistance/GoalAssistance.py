#coding=utf-8

import numpy as np

class GoalAssistance:
    """
    Goal Assistance Class \n
    Args:
        id: ID object
        pose: center pose
        grasp_points: list of grasping points 
    """
    def __init__(self, id, pose, grasp_points):
        self._id = id
        self._pose_center = pose
        self._grasp_points = grasp_points
        self._probability = 0
        self._distance = float('inf')

    def getID(self):
        """
        Get ID object associated with the goal \n
        Return: ID object associated with the goal
        """
        return self._id
    
    def getCenterPosition(self):
        """
        Get center position of object associated with the goal \n
        Return: center of object 
        """
        return self._pose_center[0:3,3]

    def getCenterMatrix(self):
        """
        Get center matrix pose of object associated with the goal \n
        Return: center matrix pose of object 
        """
        return self._pose_center

    def getGraspingPoints(self):
        """
        Get grasping points associated with the goal \n
        Return: grasping points
        """
        return self._grasp_points

    def getDistance(self):
        """
        Get distance from goal to ee_position \n
        Return: distance from goal to ee
        """
        return self._distance

    def getProbability(self):
        """
        Get probability associated with the goal \n
        Return: probability of goal
        """
        return self._probability

    def updateDistance(self, new_distance):
        """
        Update distance from goal to ee position \n
        Args:
            new_distance: new distance from goal to ee
        """
        self._distance = new_distance

    def printGoal(self):
        print("ID: " +str(self._id) )
        print("center: " +str(self._pose_center))
        print("Distance: " +str(self._distance))
        print("Probability: " +str(self._probability))  

    def updateProbability(self, new_prob):
        """
        Update probability \n
        Args:
            new_prob: new probability
        """
        self._probability = new_prob



