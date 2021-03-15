#ifndef CONTROL_MANIP_OBJECT__H
#define CONTROL_MANIP_OBJECT__H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseArray.h>

namespace control_manip{
    
    class Object{         
        public:
            Object(int obj_id, geometry_msgs::PoseStamped apriltagDetectedPose);
            ~Object() {}
            
            int getId();
            int getType();
            std::string getName();
            geometry_msgs::Pose getCenter();
            std::vector<double> getDimension();

            moveit_msgs::CollisionObject getCollisionObject();
            geometry_msgs::PoseArray getObstacle();
            geometry_msgs::PoseArray getEscapePoints();

            void createGraspingPoints(std::string frame_id);
            std::vector<geometry_msgs::PoseStamped> getGraspingPoints();
            visualization_msgs::Marker visualizeGraspingPoints();
            
        private:
            int id;     //tag's ID
            int type;   //type: cube = 0, cyl_prism = 1, triangle = 3
            std::string name;   //object name
            geometry_msgs::Pose center; //object's center
            double width;
            double length;
            double height;
            double radius; //only for cylinder
            double my_yaw;
            const std::string WORLD_FRAME = "world";

            moveit_msgs::CollisionObject collision;
            geometry_msgs::PoseArray obstacle;
            geometry_msgs::PoseArray escape_points;
            
            std::vector<geometry_msgs::PoseStamped> grasping_points_list;
            visualization_msgs::Marker marker_point;
            
            void configure(geometry_msgs::PoseStamped apriltagDetectedPose);
            geometry_msgs::PoseStamped transform(geometry_msgs::PoseStamped input_pose, std::string out_frame);
            void setCenter(geometry_msgs::PoseStamped pose);
            geometry_msgs::Quaternion setQuaternion(int index);
            void createTargetCollisionObject(std::string frame_id);
            void createObstacle();
            void createObstacleCollisionObject(std::string frame_id);
            void createEscapePoints();

    };
}

#endif

