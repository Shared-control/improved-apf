#ifndef CONTROL_MANIP_MANIPULATION_H
#define CONTROL_MANIP_MANIPULATION_H

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_manip/utils.h>
#include <control_manip/Command.h>
#include <control_manip/Move.h>
#include <control_manip/gripper.h>

#include <control_manip/JointsMove.h>
#include <control_manip/JointsCmd.h>

namespace control_manip{
    
    struct InputParameters{
        bool simulation;
        std::string robot_type;
        bool gripper_active;
        bool dynamic;
    };

    class Manipulation{
        public:            
            Manipulation(InputParameters t_params, 
                        std::shared_ptr<ros::NodeHandle> t_nh_ptr,
                        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> t_mg_ptr,
                        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> t_pli_ptr,
                        Gripper t_gripper);

            ~Manipulation() {}
            
            bool goHome();
            bool cartesianPath(geometry_msgs::Pose next_pose);
            bool pick(geometry_msgs::Pose pose);
            bool place(geometry_msgs::Pose pose);
            void addCollisionObjectToScene(moveit_msgs::CollisionObject collision_object);
            void updateCollisionScene(std::vector<moveit_msgs::CollisionObject> collision_object_vector);
            void updateCollisionScene(std::vector<moveit_msgs::CollisionObject> collision_object_vector, std::vector<moveit_msgs::ObjectColor> color_object_vector);
            std::vector<double> getCurrentJoint();

            void visualizeMarkerEscape(std::vector<geometry_msgs::PoseArray> escape_points);
            void visualizeMarkerGrasping(std::vector<geometry_msgs::PoseStamped> grasping_points);

        private:
            void configure();
            bool goJoint(std::vector<double> joints);
            bool goPose(geometry_msgs::Pose pose);
            bool moveService(control_manip::Move::Request &req, control_manip::Move::Response &res);
            
            InputParameters m_params;
            ros::NodeHandle m_nh;
            std::shared_ptr<ros::NodeHandle> m_nh_ptr;
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
            std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_scene;
            moveit::planning_interface::MoveGroupInterface::Plan m_plan;
            std::vector<std::string> m_collision_objects;
            ros::ServiceServer m_move_server;
            ros::Publisher m_pub_marker;

            const std::string PLANNING_GROUP = "manipulator";
            const std::string UR5 = "/ur5";
            const std::string UR10 = "/ur10";
            const int m_max_attempt = 100;
            const double m_jump_threshold = 0.0;
            const double m_eef_step = 0.001;
            const bool m_avoid_collision = false;
            const double m_fraction_threshold = 0.8;
            const std::vector<double> HOME_JOINTS_NO_GRIPPER = {-1.8419152908361127, -1.5043811821531188, 2.7216235258970656, -2.7514960178574217, -1.5684341877921888, -0.22205800161406497};

            Gripper m_gripper;
            Gripper::Status m_gripper_status = Gripper::Status::DEACTIVE;

            bool serviceJointsMove(control_manip::JointsMove::Request& req, control_manip::JointsMove::Response& res);
            ros::Publisher pub_joints_cmd_;
            ros::ServiceServer joints_move_srv_;

    };
}

#endif