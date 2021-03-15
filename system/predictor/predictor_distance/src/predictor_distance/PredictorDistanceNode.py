#!/usr/bin/env python
#coding=utf-8

import rospy
from shared_control_autonomy import Utils
from control_manip.msg import GoalArray as goalArrayMsg

import GoalPredictor as gp
import PredictorDistance as pd
import numpy as np


if __name__ == "__main__":
    rospy.init_node("prediction_node", anonymous=True)
    robot_type = rospy.get_param("~robot_type", "/ur5")
    name_service_init = "init_prediction_srv"

    prediction = pd.PredictorDistance(name_service_init)

    rospy.spin()


