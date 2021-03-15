#ifndef CONTROL_MANIP_GRIPPER_H
#define CONTROL_MANIP_GRIPPER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>


namespace control_manip{

    class Gripper{
        public:
            enum Status{
                OPEN,
                CLOSE,
                ACTIVE,
                DEACTIVE,
                STOP
            };

            Gripper(std::shared_ptr<ros::NodeHandle> t_nh_ptr);
            ~Gripper() {}

            Status open();
            Status close();
            Status activation();
            Status getStatus();

        private:
            std::shared_ptr<ros::NodeHandle> m_nh_ptr;
            ros::Publisher m_command_pub;
            ros::Subscriber m_status_sub;
            ros::Rate m_rate;
            
            robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInputConstPtr m_status_ptr;
            robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput m_command_msg;
            
            void configure();

    };
}
#endif