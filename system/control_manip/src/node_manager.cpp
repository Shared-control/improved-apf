#include <ros/ros.h>
#include <control_manip/manager.h>
#include <control_manip/manipulation.h>
#include <control_manip/gripper.h>
#include <ros/console.h>

int main(int argc, char **argv){

    if (ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Error)){
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ros::init(argc, argv, "control_manipulator_node");
    auto nh = std::make_shared<ros::NodeHandle>("~");

    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    control_manip::InputParameters params;
    nh->getParam("sim", params.simulation);
    nh->getParam("gripper", params.gripper_active);
    nh->getParam("robot_type", params.robot_type);
    nh->getParam("dynamic", params.dynamic);

    auto move_group_manager = std::make_shared<moveit::planning_interface::MoveGroupInterface>("manipulator");
    auto planning_scene_manager = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    control_manip::Gripper gripper(nh);

    control_manip::Manipulation manipulation(params, nh, 
                                             move_group_manager,
                                             planning_scene_manager,
                                             gripper);
    
    control_manip::Manager manager(params, nh, manipulation);
    manager.start();

    ros::waitForShutdown();
    return 0;
}