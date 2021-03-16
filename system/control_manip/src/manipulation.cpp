#include <control_manip/manipulation.h>

/**
 * Costructor of Manipulator Class
 * @param t_params Input Parameters
 * @param t_nh_ptr NodeHandle Ptr
 * @param t_mg_ptr MoveGroup Ptr
 * @param t_pli_ptr PlanningSceneInterface Ptr
 */
control_manip::Manipulation::Manipulation(InputParameters t_params, 
                        std::shared_ptr<ros::NodeHandle> t_nh_ptr,
                        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> t_mg_ptr,
                        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> t_pli_ptr,
                        Gripper t_gripper)
    : m_params(t_params)
    , m_nh_ptr(t_nh_ptr)
    , m_move_group(t_mg_ptr)
    , m_planning_scene(t_pli_ptr)
    , m_gripper(t_gripper)
{
    configure();
}


/**
 * Set move group and service server move
 */
void control_manip::Manipulation::configure(){
    m_pub_marker = m_nh_ptr->advertise<visualization_msgs::Marker>("visualization_marker", 1000);

    m_move_group->setPoseReferenceFrame("world");
    m_move_group->setMaxVelocityScalingFactor(0.1);

    m_move_server = m_nh_ptr->advertiseService("move_srv", &control_manip::Manipulation::moveService, this);

    m_cmd_urscript_pub = m_nh_ptr->advertise<std_msgs::String>("/ur5/ur_driver/URScript", 1, true);
}


/**
 * Function to move UR with URScript
 * @param next_pose
 * @return true after it has moved
 */
bool control_manip::Manipulation::moveWithScript(geometry_msgs::Pose next_pose){
    std::string base_frame_id = "base";
    std::string world = "world";

    //Transform from world frame to base frame
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped world_to_base = tf_buffer.lookupTransform(base_frame_id, world, ros::Time(0), ros::Duration(5.0));
    geometry_msgs::PoseStamped input_pose;
    input_pose.pose = next_pose;
    geometry_msgs::PoseStamped output_pose;
    tf2::doTransform(input_pose, output_pose, world_to_base);

    std::stringstream cmd;

    cmd << "def ros_cmd(): \n";
    cmd << "movep(p[" << output_pose.pose.position.x << ", " << output_pose.pose.position.y << ", " << output_pose.pose.position.z << ", "
                      << "2.9, -1.10, -0.04], a=0.05, v=0.05, r = 0.0)\n";
    cmd << "end";
    
    //std::cout << cmd.str() << std::endl;
    std_msgs::String msg;
    msg.data = cmd.str();

    m_cmd_urscript_pub.publish(msg);
    ros::Duration(1.0).sleep();

    return true;
}


/**
 * Service to move manipulator with MoveIt
 * @param req MoveRequest
 * @param res MoveResponse
 * @return true if motion has been completed, false otherwise
 */
bool control_manip::Manipulation::moveService(control_manip::Move::Request &req, control_manip::Move::Response &res){
    bool execution_success = false;
    //Movement, pick and place routine from Command
    //Command: MOVE
    if(req.command.command == req.command.MOVE){
        //execution_success = moveWithScript(req.command.to_pose);
        execution_success = cartesianPath(req.command.to_pose);
    }
    //Command: PICK
    else if(req.command.command == req.command.PICK){
        execution_success = pick(req.command.to_pose);
    }
    //Command: PLACE
    else if(req.command.command == req.command.PLACE){
        execution_success = place(req.command.to_pose);
    }
    //Command: HOME to another pick object
    else if(req.command.command == req.command.HOME){
        execution_success = goHome();
    }
    //Command: FINISH to pick object and turn off system
    else if(req.command.command == req.command.FINISH){
        std::cout << "All objects are pick, turn off system!" <<std::endl;
        res.done = true;
        return res.done;
    }

    res.done = execution_success;
    return res.done;
}


/**
 * Go to home position
 * @return true if motion has been completed, false otherwise
 */
bool control_manip::Manipulation::goHome(){
    //Control if current state is HOME state
    std::vector<double> actual_joints = m_move_group->getCurrentJointValues();
    bool comp = false;
    for(int i = 0; i < actual_joints.size(); i++){
        comp = Utils::compare(HOME_JOINTS_NO_GRIPPER.at(i), actual_joints.at(i), 0.1);
        if(!comp){
            break;         
        }
    }
    if(comp){
        return true;
    }

    bool done_i = false;
    bool done_f = false;
    std::vector<double> target_joint;    
    //Intermediate position
    target_joint = {-1.7246678511248987, -1.7996023336993616, 2.1641526222229004, -1.9401467482196253, -1.5369065443622034, -0.11355048814882451};
    if(!m_params.gripper_active){
        done_i = goJoint(target_joint);
        if(!done_i){
            ROS_ERROR_STREAM("ERROR: MOVEMENT TO INTERMEDIATE HOME POSITION FAILED!");
            return false;
        }
    }
    //Final Home position
    if(m_params.robot_type == UR5){
        if(m_params.gripper_active){
            target_joint = {-1.9337423483477991, -2.231931511555807, 2.3062686920166016, -1.626599136983053, -1.5628321806537073, -0.3018081823932093};
        }
        else{
            target_joint = HOME_JOINTS_NO_GRIPPER;
        }
    }
    else if(m_params.robot_type == UR10){
        target_joint = {-1.5443251113835759, -1.9203873133232268, 2.706002353892867, -2.358522470879209, -1.5645504145080134, -0.16228243846068402};
    }
    ros::Duration(5.0).sleep();
    done_f = goJoint(target_joint);
    if(!done_f){
        ROS_ERROR_STREAM("ERROR: MOVEMENT TO FINAL HOME POSITION FAILED!");
        return false;
    }

    //Only in a real environment
    if(!m_params.simulation){
        //Activate and close gripper
        if(m_params.gripper_active){
            m_gripper_status = m_gripper.activation();
            if(m_gripper_status == control_manip::Gripper::Status::STOP){
                ROS_ERROR_STREAM("ERROR: NO ACTIVATION GRIPPER");
                return false;
            }

            m_gripper_status = m_gripper.close();
            if(m_gripper_status == control_manip::Gripper::Status::STOP){
                ROS_ERROR_STREAM("ERROR: NO CLOSING GRIPPER");
                return false;
            }
        }    
    }
    return true;
}


/**
 * Go to next_pose using cartesian path
 * @param next_pose geometry_msgs::Pose next_pose
 * @return true if motion has been completed, false otherwise
 */
bool control_manip::Manipulation::cartesianPath(geometry_msgs::Pose next_pose){
    m_move_group->getCurrentState();
    m_move_group->setPlanningTime(10.0);
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(next_pose);
    moveit_msgs::RobotTrajectory trajectory;
    bool execution_success = false;
    double fraction = 0;
    for(int i = 0; i < m_max_attempt; i++){
        fraction = m_move_group->computeCartesianPath(waypoints, m_eef_step, m_jump_threshold, trajectory, m_avoid_collision);
        if(fraction >= m_fraction_threshold){
            break;
        }
    }
    //if(fraction < m_fraction_threshold){
    //    ROS_ERROR_STREAM("ERROR: FRACTION = " << fraction << " < " << m_fraction_threshold);
    //    return false;
    //}
    robot_trajectory::RobotTrajectory rt(m_move_group->getCurrentState()->getRobotModel(), PLANNING_GROUP);
    rt.setRobotTrajectoryMsg(*m_move_group->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool time_success = iptp.computeTimeStamps(rt);
    rt.getRobotTrajectoryMsg(trajectory);
    m_plan.trajectory_ = trajectory;
    execution_success = (m_move_group->execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!execution_success){
        ROS_ERROR_STREAM("ERROR: MOVEMENT FAILED!");
        return false;
    }
    return true;
}


/**
 * Motion planning using joints vector
 * @param joints joints vector position
 * @return true if motion has been completed, false otherwise
 */
bool control_manip::Manipulation::goJoint(std::vector<double> joints){
    m_move_group->getCurrentJointValues();
    bool plan_success = false;
    bool execution_success = false;
    m_move_group->setJointValueTarget(joints);
    for(int i = 0; i < m_max_attempt; i++){
        plan_success = (m_move_group->plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(plan_success){
            ros::Duration(0.2).sleep();
            break;
        }
        ros::Duration(0.5).sleep();
    }
    if(plan_success){
        execution_success = (m_move_group->execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execution_success){
            return false;
        }
    }
    else{
        ROS_ERROR_STREAM("All plans failed");
        return false;
    }
    return true;    
}


/**
 * Motion planning using pose msgs
 * @param pose geometry_msgs Pose msgs
 * @return true if motion has been completed, false otherwise
 */
bool control_manip::Manipulation::goPose(geometry_msgs::Pose pose){
    m_move_group->getCurrentState();
    bool plan_success = false;
    bool execution_success = false;
    m_move_group->setPoseTarget(pose);
    for(int i = 0; i < m_max_attempt; i++){
        plan_success = (m_move_group->plan(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(plan_success){
            ros::Duration(0.2).sleep();
            break;
        }
        ros::Duration(0.5).sleep();
    }
    if(plan_success){
        execution_success = (m_move_group->execute(m_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execution_success){
            return false;
        }
    }
    else{
        ROS_ERROR_STREAM("All plans failed");
        return false;
    }
    return true;  
}


/**
 * Add collision object to the scene
 * @param collision_object moveit_msgs::CollisionObject collision object
 */
void control_manip::Manipulation::addCollisionObjectToScene(moveit_msgs::CollisionObject collision_object){
    m_planning_scene->addCollisionObjects({collision_object});
}


/**
 * Update scene with new collision objects
 * @param collision_object_vector vector of moveit_msgs::CollisionObject
 */
void control_manip::Manipulation::updateCollisionScene(std::vector<moveit_msgs::CollisionObject> collision_object_vector){
    //Get ID of all collision objects into scene to remove 
    std::vector<std::string> vector_old_coll_obj_id;
    std::map<std::string, moveit_msgs::CollisionObject> c_o = m_planning_scene->getObjects();
    for(std::map<std::string, moveit_msgs::CollisionObject>::iterator iter = c_o.begin(); iter != c_o.end(); ++iter){
        vector_old_coll_obj_id.push_back(iter->second.id);
    }
    m_planning_scene->removeCollisionObjects(vector_old_coll_obj_id);
    //std::cout << "Remove old collision objects" <<std::endl;

    //Add new collision objects
    m_planning_scene->addCollisionObjects(collision_object_vector);
    //std::cout << "Add new collision objects in the scene" <<std::endl;
}


/**
 * Get current joint values
 * @return joint values
 */
std::vector<double> control_manip::Manipulation::getCurrentJoint(){
    std::vector<double> actual_joints = m_move_group->getCurrentJointValues();
    return actual_joints;
}


/**
 * Pick target object
 * @param pose pick pose
 * @return true if pick has been completed, false otherwise
 */
bool control_manip::Manipulation::pick(geometry_msgs::Pose pose){
    //Up EE
    geometry_msgs::Pose up_pose;
    up_pose = pose;
    up_pose.position.z = 1.27;
    bool execution = cartesianPath(up_pose);  
    if(!execution){
        return execution;
    }

    //Only in a real environment
    if(!m_params.simulation){
        //Open gripper to pick object
        m_gripper_status = m_gripper.getStatus();
        if(m_gripper_status != control_manip::Gripper::Status::OPEN){
            if(m_gripper_status == control_manip::Gripper::Status::DEACTIVE){
                ROS_ERROR_STREAM("ERROR: GRIPPER IS DEACTIVE");
                return false;
            }
            m_gripper_status = m_gripper.open();
            if(m_gripper_status == control_manip::Gripper::Status::STOP){
                ROS_ERROR_STREAM("ERROR: NO OPEN GRIPPER");
                return false;
            }
        }
    }

    //Down EE
    geometry_msgs::Pose final_pose;
    final_pose.position = pose.position;
    final_pose.position.z = 1.175;
    final_pose.orientation = pose.orientation;
    execution = cartesianPath(final_pose);
    if(!execution){
        return execution;
    }

    //Only in a real environment
    if(!m_params.simulation){
        //std::cout << "Close" <<std::endl;
        m_gripper_status = m_gripper.close();
        if(m_gripper_status == control_manip::Gripper::Status::STOP){
            ROS_ERROR_STREAM("ERROR: NO CLOSING GRIPPER");
            return false;
        }

        ros::Duration(1.0).sleep();

        //std::cout << "Open" <<std::endl;
        m_gripper_status = m_gripper.open();
        if(m_gripper_status == control_manip::Gripper::Status::STOP){
            ROS_ERROR_STREAM("ERROR: NO OPEN GRIPPER");
            return false;
        }
    }

    return true;
}


/**
 * Place target object
 * @param pose place pose
 * @return true if place has been completed, false otherwise
 */
bool control_manip::Manipulation::place(geometry_msgs::Pose pose){
    //Up EE
    pose.position.z = 1.27;
    bool execution = cartesianPath(pose);  
    if(!execution){
        return execution;
    }

    //Only in a real environment
    if(!m_params.simulation){
        //Close gripper
        m_gripper_status = m_gripper.getStatus();
        if(m_gripper_status != control_manip::Gripper::Status::CLOSE){
            if(m_gripper_status == control_manip::Gripper::Status::DEACTIVE){
                ROS_ERROR_STREAM("ERROR: GRIPPER IS DEACTIVE");
                return false;
            }
            m_gripper_status = m_gripper.close();
            if(m_gripper_status == control_manip::Gripper::Status::STOP){
                ROS_ERROR_STREAM("ERROR: NO CLOSING GRIPPER");
                return false;
            }
        }
    }

    return true;
}


/**
 * Show escape points in the scene
 * @param escape_points vector of points
 */
void control_manip::Manipulation::visualizeMarkerEscape(std::vector<geometry_msgs::PoseArray> escape_points){
    visualization_msgs::Marker marker_point;
    marker_point.header.frame_id = "world";
    marker_point.header.stamp = ros::Time::now();
    marker_point.ns = "Escape points";
    marker_point.action = visualization_msgs::Marker::ADD;
    marker_point.id = 0;
    marker_point.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_point.color.r = 1.0f;
    marker_point.color.a = 1.0;
    marker_point.scale.x = 0.01;
    marker_point.scale.y = 0.01;
    marker_point.scale.z = 0.01;
    for(int j = 0; j < escape_points.size(); j++){
        for(int i = 0; i < escape_points.at(j).poses.size(); i++){
            marker_point.points.push_back(escape_points.at(j).poses.at(i).position);
        }
    }
    //std::cout << "Add escape points in the scene" << std::endl;
    m_pub_marker.publish(marker_point);
}


/**
 * Show grasping points in the scene
 * @param grasping_points vector of points
 */
void control_manip::Manipulation::visualizeMarkerGrasping(std::vector<geometry_msgs::PoseStamped> grasping_points){
    visualization_msgs::Marker marker_point;
    marker_point.header.frame_id = "world";
    marker_point.header.stamp = ros::Time::now();
    marker_point.ns = "Grasping points";
    marker_point.action = visualization_msgs::Marker::ADD;
    marker_point.id = 0;
    marker_point.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_point.color.b = 1.0f;
    marker_point.color.a = 1.0;
    marker_point.scale.x = 0.01;
    marker_point.scale.y = 0.01;
    marker_point.scale.z = 0.01;
    for(int j = 0; j < grasping_points.size(); j++){   
        marker_point.points.push_back(grasping_points.at(j).pose.position);
    }
    //std::cout << "Add grasping points in the scene" << std::endl;
    m_pub_marker.publish(marker_point);
}


