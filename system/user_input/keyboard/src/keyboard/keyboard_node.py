#!/usr/bin/env python
#coding=utf-8

import rospy
import Keyboard

if __name__ == "__main__":
    rospy.init_node("keyboard_node", anonymous=True)

    rate = rospy.Rate(100)
    
    #Start keyboard publisher
    keyboard_publisher = Keyboard.Keyboard()
    
    rospy.loginfo("Keyboard_publisher_node started")
    while(not rospy.is_shutdown()):
        rate.sleep()

    keyboard_publisher.stop()
    rospy.loginfo("Keyboard_publisher_node stopped")


