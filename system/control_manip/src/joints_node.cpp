#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_manip/JointsCmd.h>

trajectory_msgs::JointTrajectory joints_msgs_;
bool publish_ = false;

void callbackJoints(const control_manip::JointsCmdConstPtr& msg){
    joints_msgs_ = msg->trajectory;
    publish_ = msg->publish;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "joints_node");
    auto nh = std::make_shared<ros::NodeHandle>("~");

    ros::Subscriber sub_joints_cmd = nh->subscribe("/cmd_joints", 1, callbackJoints);
    ros::Publisher pub_joints_cmd = nh->advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 10);

    ros::Rate rate(20);
    while(ros::ok()){
        if(publish_){
            pub_joints_cmd.publish(joints_msgs_);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}