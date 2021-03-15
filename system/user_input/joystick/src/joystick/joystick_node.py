#!/usr/bin/env python
#coding=utf-8

import rospy
import Joystick

if __name__ == "__main__":
    rospy.init_node("joystick_control_node", anonymous=True)

    rate = rospy.Rate(100)

    #Start joystick publisher
    joystick_publisher = Joystick.Joystick()

    rospy.loginfo("Joystick_publisher_node started")
    while(not rospy.is_shutdown()):
        rate.sleep()

