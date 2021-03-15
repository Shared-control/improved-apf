#!/usr/bin/env python
#coding=utf-8

import rospy
import PredictorAssistance as pa
import Utils
import numpy as np



if __name__ == "__main__":
    rospy.init_node("prediction_assistance_node", anonymous=True)
    robot_type = rospy.get_param("~robot_type", "/ur5")
    name_service_init = "init_prediction_srv"
    
    prediction_assistance = pa.PredictorAssistance(name_service_init)
    rospy.spin()


