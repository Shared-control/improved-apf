#coding=utf-8

import numpy as np
import sympy as smp
import copy
import math
import sys
import tf.transformations as transmethods
from geometry_msgs.msg import Pose, Twist, PoseArray
import Goal


def computeDistanceXY(from_position, to_position):
    """
    Compute distance in (x,y) \n
    Args:
        from_position: actual position
        to_position: target position
    Return: distance
    """
    position_from = copy.copy(from_position[0:2])
    position_to = copy.copy(to_position[0:2])

    act_pos = np.array(position_from)
    tar_pos = np.array(position_to)
    distance = np.linalg.norm(tar_pos - act_pos)
    return distance

def computeDistance(from_position, to_position):
    """
    Compute distance in (x,y,z) \n
    Args:
        from_position: actual position
        to_position: target position
    Return: distance
    """
    act_pos = np.array(from_position)
    tar_pos = np.array(to_position)
    distance = np.linalg.norm(tar_pos - act_pos)
    return distance

def computeError(x_e, x_d):
    """
    Compute error from desired and actual position \n
    Args:
        x_e: actual position
        x_d: desired position
    Return: error e = x_d - x_e
    """
    err = np.zeros(6)
    err[0:3] =  x_d[0:3] - x_e[0:3]
    return err

def twistMsgToArray(msg):
    """
    Convert twist msg into numpy array \n
    Args:
        msg: twist msg
    Return: twist numpy array
    """
    twist = np.zeros(6)
    twist[0:3] = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    twist[3:6] = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
    return twist

def arrayToTwistMsg(array):
    """
    Convert numpy array into twist message \n
    Args:
        array: numpy array
    Return: twist message 
    """
    twist = Twist()
    twist.linear.x = array[0]
    twist.linear.y = array[1]
    twist.linear.z = array[2] 
    twist.angular.x = array[3]
    twist.angular.y = array[4]
    twist.angular.z = array[5]
    return twist

def matrixToPoseMsg(matrix):
    """
    Convert a 4x4 numpy matrix to a Pose message \n
    Args:
        matrix: 4x4 numpy matrix
    Return: Pose msg
    """
    pose = Pose()
    pose.position.x = matrix[0,3]
    pose.position.y = matrix[1,3]
    pose.position.z = matrix[2,3]
    quat = transmethods.quaternion_from_matrix(matrix)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def PoseMsgToMatrix(pose_msg):
    """
    Convert Pose message to a 4x4 numpy matrix \n
    Args:
        pose_msg: Pose message
    Return: 4x4 numpy matrix
    """
    matrix = np.zeros((4,4))
    matrix[3,3] = 1
    matrix[0:3, 3] = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    q = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
    matrix[0:3, 0:3] = np.array(transmethods.quaternion_matrix(q))[0:3, 0:3]

    return matrix

def setTwist(twist, vmax):
    """
    Set limits for ee twist \n
    Args:
        twist: twist ee
        vmax: absolute value of velocity
    Return: twist
    """
    x = twist[0]
    y = twist[1]
    z = twist[2]

    module_v = math.sqrt(x**2 + y**2 + z**2)
    x = vmax * x/module_v
    y = vmax * y/module_v
    z = vmax * z/module_v
    
    #only linear components
    newTwist = np.zeros(6)
    newTwist[0] = x
    newTwist[1] = y
    newTwist[2] = z
    #newTwist[3:6] = twist[3:6]

    return newTwist

def getGoal(goal_msg):
    """
    Create and get list of Goal obj \n
    Args:
        goal_msg: msg of goal
    Return: list of goal and positions
        goal_list: list of Goal obj 
        target_pos: positions of goals
    """  
    goal_list = []
    target_pos = []
    for g in goal_msg.goal:
        grasp_points = []
        id = g.id
        center = PoseMsgToMatrix(g.center)
        #tmp_c = np.array([g.center.position.x, g.center.position.y, g.center.position.z])
        tmp_c = np.array([g.grasping_points[0].pose.position.x, g.grasping_points[0].pose.position.y, g.grasping_points[0].pose.position.z])
        target_pos.append(tmp_c)
        for grasp in g.grasping_points:
            grasp_points.append(grasp)
        goal = Goal.Goal(id, center, grasp_points)
        goal_list.append(goal)
    
    return goal_list, target_pos

def getListPoints(msg):
    """
    Get list of points(numpy vector) from msg \n
    Args:
        msg: geometry_msgs PoseArray
    Return: list of points
    """
    point_list = []
    for point in msg.poses:
        tmp = np.array([point.position.x, point.position.y, point.position.z])
        point_list.append(tmp)        
    return point_list

def getInitJoints(joints_msg):
    """
    Return numpy array of initial value of joints (vector q) \n
    Args: 
        joints_msg: joints msg
    Return: initial value of joints
    """
    joint = np.array([joints_msg.positions[0], joints_msg.positions[1], joints_msg.positions[2], 
                      joints_msg.positions[3], joints_msg.positions[4], joints_msg.positions[5]])    
    return joint
    
def getMinDistance(points_set, point_x):
    """
    Get min distance from point x to set of points (x not in the set) \n
    Args:
        points_set: list of points
        point_x: point x
    Return: minimum distance
    """
    distance = float('inf')
    for p in points_set:
        dist = computeDistance(point_x, p) 
        if(dist < distance):
            distance = dist

    return distance

def checkDimension(point1, point2):
    """
    Check if the two points are same shape and dimension (size) \n
    Args:
        point1: point 1
        point2: point 2
    Return: True if two points are same dimsnione and shape, False otherwise
    """
    if((not isinstance(point1, np.ndarray)) or (not isinstance(point2, np.ndarray))):
        print("Type of point1: " +str(type(point1)))
        print("Type of point2: " +str(type(point2)))
        print("One of the points aren't a numpy ndarray!")
        return False
    if(np.shape(point1) != np.shape(point2)):
        print("Shape of the two points is different!")
        return False
    if(np.size(point1) != np.size(point2)):
        print("Two points are different dimensions")
        return False
    return True
    
def compareVector(a, b):
    """
    Compare two numpy array (a, b) \n
    Args:
        a: numpy array
        b: numpy array
    Return: True if a = b, False otherwise
    """
    if((not isinstance(a, np.ndarray)) or (not isinstance(b, np.ndarray))):
        print("Type of a: " +str(type(a)))
        print("Type of b: " +str(type(b)))
        print("One of the points aren't a numpy ndarray!")
        return False

    comparison = a == b
    if(not comparison.all()):
        return False
    return True

def truncate(number, decimals=5):
    """
    Truncate number \n
    Args:
        number: number
        decimals: number of decimals (default = 5)
    Return: truncate number
    """
    stepper = 10.0 ** decimals
    return math.trunc(stepper * number) / stepper


