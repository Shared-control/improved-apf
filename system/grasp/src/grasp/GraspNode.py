#!/usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from control_manip.msg import Goal as goalMsg
import Grasp
import sys

if __name__ == "__main__":
    rospy.init_node("grasping_points_node", anonymous=True)
    grasp = Grasp.Grasp()

    rospy.spin()

