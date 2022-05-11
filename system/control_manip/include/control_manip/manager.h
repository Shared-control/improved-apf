#ifndef CONTROL_MANIP_MANAGER_H
#define CONTROL_MANIP_MANAGER_H

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <control_manip/object.h>
#include <control_manip/manipulation.h>
#include <control_manip/utils.h>
#include <control_manip/Command.h>
#include <control_manip/Goal.h>
#include <control_manip/GoalArray.h>
#include <control_manip/Objects.h>
#include <control_manip/Status.h>
#include <control_manip/InitObj.h>

namespace control_manip{

    class Manager{
        public:
            Manager(InputParameters t_params, 
                    std::shared_ptr<ros::NodeHandle> t_nh_ptr, 
                    control_manip::Manipulation t_manipulation);
            
            ~Manager() {}
            void start();

        private:
            bool objService(control_manip::InitObj::Request &req, control_manip::InitObj::Response &res);

            std::shared_ptr<ros::NodeHandle> m_nh_ptr;
            ros::NodeHandle m_nh;
            InputParameters m_params;
            
            //Apriltag message pointer
            apriltag_ros::AprilTagDetectionArrayConstPtr m_msg_apriltag_ptr;
            
            ros::ServiceServer m_objects_server;
            Objects m_objects_msg;
            Manipulation m_manipulation;

            void callbackApriltag(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg_april);
            ros::Publisher m_pub_objs;
            ros::Subscriber m_sub_april;

            void setObjectsParameters();

            moveit_msgs::CollisionObject createWall();

            void createCollisionForRviz();

            moveit_msgs::CollisionObject createCollisionObject(Object object);
            moveit_msgs::CollisionObject createSupportCollision(Object object);


    };
}

#endif