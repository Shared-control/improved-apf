#coding=utf-8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from joystick.msg import JoyCommand

class Joystick:
    def __init__(self):
        self.sub_joy = rospy.Subscriber("joy", Joy, self.callback)
        self.pub_command = rospy.Publisher("joy_command", JoyCommand, queue_size=1)

        self.twist_msg = Twist()
        self.command_msg = JoyCommand()

    def callback(self, joy_msg):
        self.command_msg.header.stamp = rospy.Time.now()

        #Create zero twist message
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        #Manage command message
        X_button = joy_msg.buttons[0]
        O_button = joy_msg.buttons[1]
        T_button = joy_msg.buttons[2]
        S_button = joy_msg.buttons[3]

        #COMMAND
        # X = PICK
        # O = PLACE
        # Triangle = HOME
        # Square = FINISH
        # Left and right Sticks, L1, L2, R1, R2 = TWIST
                
        if(X_button != 0):
            self.command_msg.command = self.command_msg.PICK
        
        elif(O_button != 0):
            self.command_msg.command = self.command_msg.PLACE
        
        elif(T_button != 0):
            self.command_msg.command = self.command_msg.HOME

        elif(S_button != 0):
            self.command_msg.command = self.command_msg.FINISH

        else:
            self.command_msg.command = self.command_msg.TWIST

            #For twist linear (x, y) used left stick
            self.twist_msg.linear.x = joy_msg.axes[0]
            self.twist_msg.linear.y = - joy_msg.axes[1]
            #For twist linear z used L1 and R1 buttons: L1 = -1.0, R1 = 1.0
            self.twist_msg.linear.z = - joy_msg.buttons[4] + joy_msg.buttons[5]

            #For twist angular (x, y) used right stick
            self.twist_msg.angular.x = joy_msg.axes[3]
            self.twist_msg.angular.y = joy_msg.axes[4]
            #For twist angular z used L2 and R2 buttons: L2 = -1.0, R2 = 1.0
            self.twist_msg.angular.z = - joy_msg.buttons[6] + joy_msg.buttons[7]

            self.command_msg.twist = self.twist_msg

        #Publish command message
        self.pub_command.publish(self.command_msg)

