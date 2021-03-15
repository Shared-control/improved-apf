#coding=utf-8

import sympy as smp
import math
import random
import time
import numpy as np 
import Utils
import sys
import Csp
import PrintFile

class PotentialFunction:
    """
    Args:
        threshold_distance: threshold distance p0
        neta: repulsive constant
        k_a: attractive constant
        escape_constant: escape constant
        obstacle_position: list of obstacles positions
        rob_kin: RobotKinematic object
        init_q: initial value of angles
        goal: goal position
        escape_points: list of escape points
        print_file: PrintFile object
        gripper_active: True if there is a gripper, False otherwise
    """
    def __init__(self, threshold_distance, neta, k_a, escape_constant, obstacle_position, rob_kin, init_q, goal, escape_points, print_file, gripper_active, escape_active):
        self._actual_positions = []
        self._obs_position = obstacle_position
        self._p0 = threshold_distance
        self._neta = neta
        self._rob_kin = rob_kin
        self._actual_q = init_q
        self._goal_position = goal
        self._k_a = k_a
        self._escape_points = escape_points
        self._escape_constant = escape_constant
        self._indexEE = len(self._actual_q)
        self._print_file = print_file
        self._gripper_active = gripper_active
        self._escape_active = escape_active
        self._LEN_TWIST_VECTOR = 6

        self.csp = Csp.CSP()


    def updatePosition(self, new_q):
        """
        Update joints and EE positions from actual angles q
        Args:
            new_q: actual angles q
        """
        self._actual_q = new_q
        new_positions = []
        for i in range(len(self._actual_q)):
            tmp = self._rob_kin.get_pose(self._actual_q, i)
            new_positions.append(tmp[0:3,3])
        
        #If gripper is active, then self._actual_position[self._indexEE] is the position of control point of the gripper,
        #Otherwise self._actual_position[self._indexEE] is the position of control point of the EE
        if(self._gripper_active):
            tmp = self._rob_kin.get_pose(self._actual_q, self._indexEE + 1)
            new_positions.append(tmp[0:3,3])
        else:
            tmp = self._rob_kin.get_pose(self._actual_q, self._indexEE)
            new_positions.append(tmp[0:3,3])
        
        #Update positions
        self._actual_positions = new_positions


    def getAttractivePotential(self, goal_distribution):
        """
        Compute Goals attractive force only for EE:
            f(x) = -grad_U_a(x) = k_a * (x_goal - x_ee)/p(x)
            p(x) = distance from actual position for EE to goal position
        Args:
            goal_distribution: list of probabilities of goals
        Return: attractive force for EE to goal
            attr_pot = attractive force for EE to goal 
        """
        ee_position = self._actual_positions[self._indexEE]
        total_attr = np.zeros(self._LEN_TWIST_VECTOR)
        for i in range(len(self._goal_position)):
            distance = Utils.computeDistance(ee_position, self._goal_position[i])
            attr_pot = np.zeros(self._LEN_TWIST_VECTOR)
            #only linear component
            attr_pot[0:3] = goal_distribution[i] *(self._k_a/distance)*(self._goal_position[i][0:3] - ee_position[0:3])
            total_attr += attr_pot 

        return total_attr


    def getRepulsivePotential(self, distance_from_obstacle, idobs, idpos):
        """
        Compute repulsive force:
            f(x) = -grad_U_r(x) = neta * (1/p(x) - 1/p0) * (1/(p(x)^2)) * (x-x_obs)/p(x)  
        Args:
            distance_from_obstacle: distance of actual joint position from obstacle [p(x)]
            idobs: id obstacle closest to actual joint
            idpos: actual joint 
        Return: repulsive force
            rep_f = repulsive force
        """
        p_x = distance_from_obstacle
        rep_f = np.zeros(self._LEN_TWIST_VECTOR)
        ft = 1/p_x
        st = 1/self._p0
        #only linear component
        rep_f[0:3] = self._neta * (ft - st) * (ft * ft) * ((self._actual_positions[idpos][0:3] - self._obs_position[idobs][0:3])/p_x)
        
        return rep_f


    def checkRange(self, goal_distribution):
        """
        Args:
            goal_distribution: list of probabilities
        """
        ee_position = self._actual_positions[self._indexEE]
        x_ee, y_ee, z_ee = self.getSingleCoordiante(ee_position)
        
        #only the goal that has the highest probability
        prob_max = max(goal_distribution)
        idx_goal_max_prob = goal_distribution.index(prob_max)
        closest_goal = self._goal_position[idx_goal_max_prob]
        if(not Utils.checkDimension(ee_position, closest_goal)):
            sys.exit()
        
        #Same dimension
        dimension_point = np.size(ee_position)
        x_cg, y_cg, z_cg = self.getSingleCoordiante(closest_goal)
        x_range = self.getInterval(x_ee, x_cg)
        y_range = self.getInterval(y_ee, y_cg)
        z_range = self.getInterval(z_ee, z_cg)
        
        tmp_list = list()
        for obs in self._obs_position:
            flag = 0
            for i in range(dimension_point):
                ax_obs = obs[i]
                if(i == 0):
                    if(x_range[0] <= ax_obs <= x_range[1]):
                        flag += 1 
                elif(i == 1):
                    if(y_range[0] <= ax_obs <= y_range[1]):
                        flag += 1
                elif(i == 2):
                    if(z_range[0] <= ax_obs <= z_range[1]):
                        flag += 1
                else:
                    print("ERROR DIMENSION!")
                    sys.exit()
            if(flag == 3):
                return False

        return True


    def getInterval(self, a, b):
        if(a < b):
            return [a, b]
        return [b, a]


    def getSingleCoordiante(self, position):
        if(np.size(position) != 3):
            print("Error: array haven't 3 dimension!")
            sys.exit()
        x = position[0]
        y = position[1]
        z = position[2]

        return x, y, z


    def createConstraints(self, goal_distribution):
        """
        Create lists of constraints C1, C2, C3. 
        Args: \n
        Return: lists of constraints C1, C2, C3
        """
        c1 = self.computeC1(goal_distribution)
        c2 = self.computeC2()
        c3 = self.computeC3()

        self._print_file.write_with_title(c1, "C1")
        self._print_file.write_with_title(c2, "C2")
        self._print_file.write_with_title(c3, "C3")

        return c1, c2, c3


    def computeC1(self, goal_distribution):
        """
        Compute constraints C1 \n
        C1: tuple (id goal, probability) \n
        Args:
            goal_distribution: list of probabilities
        Return: list of constraints C1
        """
        c1_list = list()

        for gi, prob in enumerate(goal_distribution):
            tmp_tuple = tuple((gi, prob))
            c1_list.append(tmp_tuple)

        return c1_list


    def computeC2(self):
        """
        Compute constraints C2 \n
        C2: tuple (id escape point, value) \n
        value = number in [0,1] of the reciprocal distance from EE to escape point
        Args: \n
        Return: list of constraints C2
        """
        c2_list = list()
        recip_list_ee = list()
        max_value_num = -1
        min_value_num = float('inf')
        ee_position = self._actual_positions[self._indexEE]
        
        #Compute reciprocal of the distance and the element [id escape point, reciprocal], then insert this element in recip_list_ee
        for ei, esp in enumerate(self._escape_points):
            dist_ee_esp = Utils.computeDistance(ee_position, esp)
            recip_dist_ee_esp = 1./dist_ee_esp
            recip_list_ee.append([ei, recip_dist_ee_esp])

            #Search max and min
            if(recip_dist_ee_esp < min_value_num):
                min_value_num = recip_dist_ee_esp
            if(recip_dist_ee_esp > max_value_num):
                max_value_num = recip_dist_ee_esp
        
        max_min_value = max_value_num - min_value_num
        
        #MAP in [0,1] the reciprocal in the element [id escape point, reciprocal], then create the tuple(id escape point, value)
        for i in recip_list_ee:
            value = (i[1] - min_value_num) / max_min_value
            tmp_tuple = tuple((i[0], value))
            c2_list.append(tmp_tuple)

        return c2_list


    def computeC3(self):
        """
        Compute constraints C3  \n
        C3: tuple (id goal, id escape p, value) \n
        value = number in [0,1] of the reciprocal of the distance from goal to escape point \n
        Args: \n
        Return: list of constraints C3
        """
        recip_list = list()
        c3_list = list()
        max_num = -1
        min_num = float('inf')
        
        #Compute reciprocal of the distance and the element [id goal, id escape point, reciprocal], then insert this element in recip_list
        for gi, gp in enumerate(self._goal_position):
            for ei, ep in enumerate(self._escape_points):
                dist_g_esp = Utils.computeDistance(gp, ep)
                recip_dist_g_esp = 1./dist_g_esp
                recip_list.append([gi, ei, recip_dist_g_esp])
                
                #Search max and min
                if(recip_dist_g_esp < min_num):
                    min_num = recip_dist_g_esp
                if(recip_dist_g_esp > max_num):
                    max_num = recip_dist_g_esp

        max_min = max_num - min_num

        #Map in [0,1] the reciprocal in the element [id goal, id escape point, reciprocal], then create the tuple(id goal, id escape point, value)
        for i in recip_list:
            value = (i[2] - min_num) / max_min
            tmp_tuple = tuple((i[0], i[1], value))
            c3_list.append(tmp_tuple)

        return c3_list


    def getCATwist(self, new_q, vmax, goal_distribution):
        twist_q = self.getTotalPotential(new_q, goal_distribution)
        twist_ca = np.zeros(self._LEN_TWIST_VECTOR)
        jacob_matr = self._rob_kin.evaluateJacobian(new_q, self._indexEE)
        twist_ca = jacob_matr.dot(twist_q)
        twist_ca = 2*Utils.setTwist(twist_ca, vmax)
        
        return twist_ca


    def getTotalPotential(self, new_q, goal_distribution):
        total_force = np.zeros(self._LEN_TWIST_VECTOR)
        rep_force_ee = np.zeros(self._LEN_TWIST_VECTOR)
        total_rep_force = np.zeros(self._LEN_TWIST_VECTOR)

        #compute actual positions of all joints and EE from configuration new_q
        self.updatePosition(new_q)

        #for each control points check if it is under influence of potential field
        for ap in range(len(self._actual_positions)):
            #use to check distance from actual control point and the closest obstacle
            min_distance = float('inf')
            #obstacle id in obs_position list that is closest to the actual control point
            idmin = None
            #find the nearest obstacle and save id
            for obs in range(len(self._obs_position)):
                distance = Utils.computeDistance(self._actual_positions[ap], self._obs_position[obs])
                if(distance < min_distance):
                    min_distance = distance
                    idmin = obs
            #check if the control point is under influence of potential field and compute repulsive force
            if(min_distance <= self._p0):
                #Repulsive force for EE
                if(ap == self._indexEE):
                    rep_force_ee = self.getRepulsivePotential(min_distance, idmin, ap)
                #Repulsive force for all other points, in this case joints
                else:
                    rep_force = self.getRepulsivePotential(min_distance, idmin, ap)
                    jacobian = self._rob_kin.evaluateJacobian(new_q, ap)
                    jac_T = np.transpose(jacobian) 
                    total_rep_force += np.dot(jac_T, rep_force)

        #EE components: Attractive + Repulsive
        #Compute jacobian and its transpose
        jac_ee = self._rob_kin.evaluateJacobian(new_q, self._indexEE)
        jac_ee_T = np.transpose(jac_ee) 
        
        #Attractive goals
        att_force_ee = self.getAttractivePotential(goal_distribution)
        
        #Compute total force on EE
        total_force_ee = np.zeros(self._LEN_TWIST_VECTOR)
        
        #Attractive escape potential:
        #Activate escape points only if there is an obstacle between the current position and the most probable goal
        attr_escape_p = np.zeros(self._LEN_TWIST_VECTOR)
        if((not self.checkRange(goal_distribution)) and (self._escape_active)):
            print("ESCAPE")
            attr_escape_p = self.getEscapePotential(goal_distribution)

        total_force_ee = att_force_ee + rep_force_ee + attr_escape_p
        total_ee = np.dot(jac_ee_T, total_force_ee)   
        
        #Final force
        total_force = total_ee + total_rep_force

        return total_force


    def getEscapePotential(self, goal_distribution):
        ee_position = self._actual_positions[self._indexEE]
        total_attr_escape = np.zeros(self._LEN_TWIST_VECTOR)
        
        list_c1, list_c2, list_c3 = self.createConstraints(goal_distribution)

        result_csp = self.csp.fuzzySCSP(list_c1, list_c2, list_c3)
        #print("RESULT ESCAPE: " +str(result_csp))

        self._print_file.write_with_title(result_csp, "output_csp")

        id_esp = result_csp[1]
        pos_esp = self._escape_points[id_esp]
        #print("ID ESP: " +str(id_esp))

        total_attr_escape[0:3] = self._escape_constant * (pos_esp[0:3] - ee_position[0:3])
        #print("ATTR POT: " +str(att_pot))

        return total_attr_escape
        

    def updateParams(self, new_obstacles, new_escape):
        """
        Update objects position \n
        Args:
            new_obstacles: list of obstacles
            new_escape: list of escape points
        """
        self._obs_position = new_obstacles
        self._escape_points = new_escape


    def getDynamicCATwist(self, new_q, vmax, goal_distribution, new_obstacles, new_escape):
        self.updateParams(new_obstacles, new_escape)
        
        twist_q = self.getTotalPotential(new_q, goal_distribution)
        twist_ca = np.zeros(self._LEN_TWIST_VECTOR)
        jacob_matr = self._rob_kin.evaluateJacobian(new_q, self._indexEE)
        twist_ca = jacob_matr.dot(twist_q)
        twist_ca = 2*Utils.setTwist(twist_ca, vmax)
        
        return twist_ca
