#coding=utf-8

import numpy as np
from shared_control_autonomy.Goal import Goal

class GoalPredictor(object):
    """
    Class goal predictor to compute probability distribution \n
    Args:
        goals: Goal list
    """
    def __init__(self, goals):
        self._goals = goals
        self._goal_distribution = (1./len(self._goals)) * np.ones(len(self._goals)) 
        
    def updateDistribution(self):
        """
        Update distribution probability after new action
        """
        if(len(self._goal_distribution) == 0):
            return   

        elif(len(self._goal_distribution) == 1):
            self._goals[0].updateProbability(1)
            return   

        #Compute sum of all reciprocal distances
        sum_distance_recip = 0
        #reciprocal distance array
        reciprocal = np.zeros(len(self._goals))
        for i in range(len(self._goals)):
            reciprocal[i] += 1./self._goals[i].getDistance()
            #print("ID: " + str(self.goals[i].getID()) + " distance: " +str(self.goals[i].getDistance()))
            sum_distance_recip += reciprocal[i]
            
        self._goal_distribution = reciprocal/sum_distance_recip 
        for i in range(len(self._goal_distribution)):
            self._goals[i].updateProbability(self._goal_distribution[i])

    def getDistribution(self):
        """
        Get distribution \n
        Return: goal distribution
        """
        return self._goal_distribution

    def getMaxProbability(self):
        """
        Get max probability \n
        Return: max probability
        """
        idx_max = np.argmax(self._goal_distribution)
        max_prob = self._goal_distribution[idx_max]
        
        return max_prob

    def getIndexMaxProb(self):
        """
        Get index of goal with max probability \n
        Return: index of goal
        """
        idx_max = np.argmax(self._goal_distribution)
        return idx_max 

    def printDistribution(self):
        """
        Print distribution probability
        """
        for idx, prob in enumerate(self._goal_distribution):
            print("Goal: " + str(self._goals[idx].getID()) + " with prob: " + str(prob))



