#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <control_manip/Command.h>
#include <control_manip/Goal.h>
#include <control_manip/GoalArray.h>
#include <control_manip/Objects.h>
#include <control_manip/Status.h>



namespace control_manip{
    namespace Utils{
        double computeDistance(geometry_msgs::Pose from, geometry_msgs::Pose to);
        geometry_msgs::Pose createPose(double x, double y, double z, double ox, double oy, double oz, double ow);
        bool compare(double a, double b, double difference);
        Goal createGoalMsg(int id, geometry_msgs::Pose center, std::vector<geometry_msgs::PoseStamped> grasping_points);
        GoalArray createGoalArrayMsg(std::vector<Goal> goals);
        Objects createObjectsMsg(std::vector<Goal> goals, std::vector<geometry_msgs::PoseArray> obstacles, std::vector<double> joints, std::vector<geometry_msgs::PoseArray> escape_points);
    }
}