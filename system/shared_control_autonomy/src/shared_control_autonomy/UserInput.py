#coding=utf-8

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

from keyboard.msg import KeyCommand
from joystick.msg import JoyCommand
from myo.msg import MyoCommand

import sys
import time
import numpy as np 
import copy

class UserInput:
    """
    Class to manage User Input Command \n
    Args:
        robot type: robot type
        user_input_type: type of user input
    """
    def __init__(self, robot_type, user_input_type):
        self._robot_type = robot_type
        self._user_input_type = user_input_type
        self._name_topic_user_command = robot_type + "/user_command"
        
        #Subscriber user command
        if(self._user_input_type == "myo"):
            rospy.Subscriber(self._name_topic_user_command, MyoCommand, self.callbackUserMyoCommand)
        elif(self._user_input_type == "joystick"):
            rospy.Subscriber(self._name_topic_user_command, JoyCommand, self.callbackUserJoyCommand)
        else:
            rospy.Subscriber(self._name_topic_user_command, KeyCommand, self.callbackUserKeyCommand)

        self._twist_user = np.zeros(6)
        self._twist_user_time = rospy.Time.now()
        self._command = None

    
    def callbackUserMyoCommand(self, msg):
        if(msg.command == MyoCommand.TWIST):
            self.setTwist(msg.twist)
        self._command = msg.command
    
    
    def callbackUserJoyCommand(self, msg):
        if(msg.command == JoyCommand.TWIST):
            self.setTwist(msg.twist)
        self._command = msg.command

    
    def callbackUserKeyCommand(self, msg):
        if(msg.command == KeyCommand.TWIST):
            self.setTwist(msg.twist)
        self._command = msg.command

    
    def setTwist(self, twist):
        #twist with linear and angular components
        self._twist_user[0:3] = 10 * np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        self._twist_user[3:6] = 10 * np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        
    
    def getTwist(self):
        return copy.copy(self._twist_user) 

    
    def getCommand(self):
        return copy.copy(self._command)

