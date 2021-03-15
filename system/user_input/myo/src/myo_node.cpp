#include <ros/ros.h>
#include <myo/myo_controller.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "myo_control_node");
    ros::NodeHandle nh;

    std::string axis[3];
    //For now
    axis[0] = "p-"; 
    axis[1] = "y+"; 
    axis[2] = "r+";  
    
    /*
    axis[0] = "r+"; //roll
    axis[1] = "p-"; //pitch
    axis[2] = "y+"; //yaw 
    */
    std::vector<int> threshold_angle = {15, 15, 15};

    myo::MyoController myo_control(nh, axis, threshold_angle);

    ros::spin();
    return 0;
}