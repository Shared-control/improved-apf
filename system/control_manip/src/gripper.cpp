#include <control_manip/gripper.h>

/**
 * Costructor of Gripper Class 
 * @param t_nh_ptr NodeHandle Ptr
 */
control_manip::Gripper::Gripper(std::shared_ptr<ros::NodeHandle> t_nh_ptr)
    : m_nh_ptr(t_nh_ptr)
    , m_rate(1)
{
    configure();
}


/**
 * Configure publisher to Activate, Open and Close gripper
 */
void control_manip::Gripper::configure(){
    m_command_pub = m_nh_ptr->advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotOutput", 1);
}


/**
 * Activate gripper
 * @return Status::ACTIVE if it is active, Status::STOP if activation fails.
 */
control_manip::Gripper::Status control_manip::Gripper::activation(){
    m_command_msg.rACT = 1;
    m_command_pub.publish(m_command_msg);

    m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
    while(m_status_ptr->gIMC != 3){
        if((m_status_ptr->gSTA == 1) || (m_status_ptr->gSTA == 2)){
            ROS_ERROR_STREAM("ACTIVE GRIPPER ERROR: FINGERS STOPPED BEFORE REQUESTED POSITION");
            return control_manip::Gripper::Status::STOP;
        }
        m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
        m_rate.sleep();
    }

    return control_manip::Gripper::Status::ACTIVE;    
}


/**
 * Open gripper
 * @return Status::OPEN if it is open, Status::STOP if opening fails.
 */
control_manip::Gripper::Status control_manip::Gripper::open(){
    m_command_msg.rACT = 1;
    m_command_msg.rGTO = 1;
    m_command_msg.rPRA = 0;
    m_command_msg.rFRA = 0;
    m_command_msg.rSPA = 200;
    m_command_pub.publish(m_command_msg);

    m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
    while((m_status_ptr->gPRA != 0) && (m_status_ptr->gSTA != 3)){
        if((m_status_ptr->gSTA == 1) || (m_status_ptr->gSTA == 2)){
            ROS_ERROR_STREAM("OPEN GRIPPER ERROR: FINGERS STOPPED BEFORE REQUESTED POSITION");
            return control_manip::Gripper::Status::STOP;
        }

        m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
        m_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    return control_manip::Gripper::Status::OPEN; 
}


/**
 * Close gripper
 * @return Status::CLOSE if it is closed, Status::STOP if closure fails.
 */
control_manip::Gripper::Status control_manip::Gripper::close(){
    m_command_msg.rACT = 1;
    m_command_msg.rGTO = 1;
    m_command_msg.rPRA = 250;
    m_command_msg.rSPA = 200;
    m_command_msg.rFRA = 200;
    m_command_pub.publish(m_command_msg);

    m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
    while((m_status_ptr->gPRA != 250) && (m_status_ptr->gSTA != 3)){
        if((m_status_ptr->gSTA == 1) || (m_status_ptr->gSTA == 2)){
            ROS_ERROR_STREAM("CLOSE GRIPPER ERROR: FINGERS STOPPED BEFORE REQUESTED POSITION");
            return control_manip::Gripper::Status::STOP;
        }

        m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
        m_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    return control_manip::Gripper::Status::CLOSE; 
}


/**
 * Get status of the gripper
 * @return actual Status: DEACTIVE, OPEN or CLOSE
 */
control_manip::Gripper::Status control_manip::Gripper::getStatus(){
    m_status_ptr = ros::topic::waitForMessage<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput>("/robotiq_hands/l_hand/Robotiq3FGripperRobotInput");
    
    if(m_status_ptr->gACT == 0){
        return control_manip::Gripper::Status::DEACTIVE;
    }
    
    //gPRA = 0 gripper is open, gPRA = 250 gripper is close
    if(m_status_ptr->gPRA == 250){
        return control_manip::Gripper::Status::CLOSE;
    }
    
    return control_manip::Gripper::Status::OPEN;
}

