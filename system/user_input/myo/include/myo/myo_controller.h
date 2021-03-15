#ifndef MYO_MYO_CONTROLLER_H
#define MYO_MYO_CONTROLLER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <myo/MyoCommand.h>
#include <myo/ResetMyo.h>
#include <sensor_msgs/Imu.h>
#include <rosmyo/ClassifierPose.h>

#include <iostream>
#include <fstream>

namespace myo{

    class MyoController{
        public:
            MyoController(ros::NodeHandle t_nh, std::string t_axis [], std::vector<int> t_threshold);
            ~MyoController() {}

        private:
            ros::NodeHandle m_nh;
            void configure(std::string t_axis []);
            
            tf2::Quaternion q_start_offset;
            tf2::Quaternion q_now;
            int follow;
            int m_axis [3]; //maps roll pitch yaw to axis x y z; roll=0, pitch=1, yaw =2, not used=-1
            int sign [3]; //inverts direction over related axis if setted to -1
            std::vector<int> m_angle_th; //threshold over axis in degree, axis0 = x
            double b_roll, b_pitch, b_yaw;
            
            MyoCommand command_msg;
            geometry_msgs::Twist twist_msg;

            ros::Publisher pub_command;
            ros::Subscriber sub_imu;
            ros::Subscriber sub_gesture;
            ros::ServiceServer m_reset_server;

            //subscriber
            void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);
            void callbackGesture(const rosmyo::ClassifierPose::ConstPtr& msg);

            //publisher
            void publishCommand();

            double deltaDeg(double current, double centre);
            void setTwist();

            void reset_msg();
            bool resetMyoService(myo::ResetMyo::Request &req, myo::ResetMyo::Response &res);
    };

} 



#endif