#coding=utf-8

import numpy as np
import sys


class CSP:
    """
    CSP class
    """
    def __init__(self):
        self._goals = list()
        self._escape_points = list()
        self._constraints_arc = list()


    def setGoals(self, goals):
        """
        id_goal -> probability \n
        self._goals is a list of tuple(id_goal, probability) \n
        Args:
            goals: list of tuple(id_goal, probability)
        """
        self._goals = goals


    def setEscapePoints(self, escape_p):
        """
        id_escape_point -> value = number in [0,1] of the reciprocal of the distance from EE to escape point \n
        self._escape_points is a list of tuple(id_escape_p, value) \n
        Args:
            escape_p: list of tuple(id_escape_p, value)
        """
        self._escape_points = escape_p


    def setConstraintsArc(self, distances):
        """
        (id_goal, id_escape_point) -> value = number in [0,1] of the reciprocal of the distance from goal to escape point \n
        self._constraints_arc is a list of tuple(id_goal, id_escape_p, value) \n
        Args:
            distances: list of tuple(id_goal, id_escape_p, value)
        """
        self._constraints_arc = distances
    

    def fuzzySCSP(self, goals, escape_p, distances):
        """
        Start fuzzy soft CSP \n
        Args:
            goals: list of tuple(id_goal, probability)
            escape_p: list of tuple(id_escape_p, value)
            distances: list of tuple(id_goal, id_escape_p, value)
        Return: tuple (id_goal, id_escape_point, value) result of projection
        """
        #Set constraints
        self.setGoals(goals)
        self.setEscapePoints(escape_p)
        self.setConstraintsArc(distances)

        #Combining
        result_comb = self.fuzzyCombining()
        #Projection
        result_proj = self.fuzzyProjection(result_comb)

        return result_proj


    def fuzzyCombining(self):
        """
        Combining = take min value \n
        Return: list of results of combining
        """
        #result_combining is a list of tuple(id_goal, id_escape_p, min) 
        results_combining = list()
        for c1 in self._goals:
            id_goal = c1[0]
            prob_goal = c1[1]
            for c2 in self._escape_points:
                id_escape_p = c2[0]
                value_escape_p = c2[1]
                #Compute c3 value
                value_constraint = self.findC3Value(id_goal, id_escape_p)
                if(value_constraint is None):
                    print("Error: c3 value is NONE")
                    sys.exit()
                #Compute min
                min_value = min(prob_goal, value_escape_p, value_constraint)
                #create tuple (id_goal, id_escape_p, min) and insert it into results_combining list
                tmp_tuple = (id_goal, id_escape_p, min_value)
                results_combining.append(tmp_tuple)

        return results_combining
    

    def fuzzyProjection(self, combining_result):
        """
        Projection = take max value \n
        Return: tuple (id_goal, id_escape_point, value) result of projection
        """
        tmp_max = -1
        final_tuple = None
        for tpe in combining_result:
            if(tpe[2] > tmp_max):
                tmp_max = tpe[2]
                final_tuple = tpe
        
        return final_tuple


    def findC3Value(self, id_g, id_ep):
        """
        C3 is the tuple in self._constraints_arc in the form (id_goal, id_escape_p, value) \n
        Args:
            id_g: id of the goal
            id_ep: id of the escape point
        Return: value of c3 with (id_g, id_ep)
        """    
        for c3 in self._constraints_arc:
            if((c3[0] == id_g) and (c3[1] == id_ep)):
                c3_value = c3[2]
                return c3_value

        return None

    