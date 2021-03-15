#include <myo/myo_controller.h>

/**
 * Costructor of MyoController class
 * @param t_nh NodeHandle
 * @param t_axis axis
 * @param t_threshold threshold
 */
myo::MyoController::MyoController(ros::NodeHandle t_nh, std::string t_axis [], std::vector<int> t_threshold)
    : m_nh(t_nh)
    , m_angle_th(t_threshold)
{
    configure(t_axis);
}

/**
 * Set axis, Publisher and Subscribers
 * @param t_axis axis
 */
void myo::MyoController::configure(std::string t_axis []){
    follow = 0;

    for(int i = 0; i < 3; i++){
        //map axis i to correct angle
        char ax_angle = t_axis[i][0];
        char ax_direction = t_axis[i][1];
        
        if(ax_angle == 'r'){
            m_axis[i] = 0;
        }
        else if(ax_angle == 'p'){
            m_axis[i] = 1;
        }
        else if(ax_angle == 'y'){
            m_axis[i] = 2;
        }
        else{
            m_axis[i] = -1; //not used
        }

        sign[i] = 1;
        if(ax_direction == '-'){
            sign[i] = -1;
        }
    }

    //Set Publisher and Subscribers
    pub_command = m_nh.advertise<myo::MyoCommand>("myo_command", 1);
    sub_imu = m_nh.subscribe("imu", 20, &myo::MyoController::callbackImu, this);
    sub_gesture = m_nh.subscribe("gesture", 20, &myo::MyoController::callbackGesture, this);

    m_reset_server = m_nh.advertiseService("reset_myo_srv", &myo::MyoController::resetMyoService, this);
    
    ROS_INFO_STREAM("Myo Controller started!");
}

/**
 * Callback Imu
 * @param msg sensor_msgs::Imu::ConstPtr&
 */
void myo::MyoController::callbackImu(const sensor_msgs::Imu::ConstPtr& msg){
    if(follow == 0){
        reset_msg();
        return;
    }

    //If no starting offset, save this as offset
    if(follow == 1){
        q_start_offset = tf2::Quaternion(msg->orientation.x,
                                         msg->orientation.y,
                                         msg->orientation.z,
                                         msg->orientation.w);
        q_start_offset.normalize();
        follow = 2;
        ROS_INFO_STREAM("Publishing started");
    }
    tf2::Quaternion now(msg->orientation.x,
                        msg->orientation.y,
                        msg->orientation.z,
                        msg->orientation.w);
    now.normalize();
    q_now = now;

    command_msg.command = command_msg.TWIST;
    publishCommand();
}

/**
 * Callback gesture
 * @param msg rosmyo::ClassifierPose::ConstPtr&
 */
void myo::MyoController::callbackGesture(const rosmyo::ClassifierPose::ConstPtr& msg){
    //If gesture is DOUBLE_TAP then modified offset
    if(msg->pose == msg->DOUBLE_TAP){
        if(follow == 2){
            //Stop publishing
            follow = 0; 
            ROS_INFO_STREAM("Publishing stopped");
        }
        else{
            //Restart publishing
            follow = 1;
            ROS_INFO_STREAM("Publishing waiting first message");
        }
        return;
    }

    if(msg->pose == msg->FIST){
        command_msg.command = command_msg.PICK;
    }
    else if(msg->pose == msg->FINGERS_SPREAD){
        command_msg.command = command_msg.PLACE;
    }
    else if(msg->pose == msg->WAVE_IN){
        command_msg.command = command_msg.HOME;
    }
    else if(msg->pose == msg->WAVE_OUT){
        command_msg.command = command_msg.FINISH;
    }
    publishCommand();
}

/**
 * Set delta deg
 * @param current
 * @param centre
 * @return delta
 */
double myo::MyoController::deltaDeg(double current, double centre){
    double delta = current - centre;
    if(delta > 180){
        delta = delta - 360;
    }
    else if(delta < -180){
        delta = delta + 360;
    }
    return delta;
}

/**
 * Publish MyoCommand message
 */
void myo::MyoController::publishCommand(){
    //Manage command message
    command_msg.header.stamp = ros::Time::now();
    
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;

    //Set twist
    if(command_msg.command == command_msg.TWIST){
        setTwist();
        command_msg.twist = twist_msg;
    }

    //Publish command message
    pub_command.publish(command_msg);
}

/**
 * Set twist for myo command message
 */
void myo::MyoController::setTwist(){
    //Use the last q_now to publish twist-command message
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_now).getRPY(roll, pitch, yaw);
    
    double ro, po, yo;
    tf2::Matrix3x3(q_start_offset).getRPY(ro, po, yo);

    double dRoll = deltaDeg(roll * 180/M_PI, ro * 180/M_PI);
    double dPitch = deltaDeg(pitch * 180/M_PI, po * 180/M_PI);
    double dYaw = deltaDeg(yaw * 180/M_PI, yo * 180/M_PI);

    double angles [] {dRoll, dPitch, dYaw};

    geometry_msgs::Twist tmp_twist;

    //For now use these settings
    //If m_axis[i] < 0 axis i is not used
    //Z axis
    if(m_axis[0] >= 0){
        //takes the correct angle from r,p,y indicated previously in m_axis vector
        double angle_z = angles[m_axis[0]];
        if(angle_z > m_angle_th[0]){
            tmp_twist.linear.z = -sign[0] * 0.15;
        }
        else if(angle_z < -m_angle_th[0]){
            tmp_twist.linear.z = sign[0] * 0.15;
        }
    }

    //Y axis
    if(m_axis[1] >= 0){
        //takes the correct angle from r,p,y indicated previously in m_axis vector
        double angle_x = angles[m_axis[1]];
        if(angle_x > m_angle_th[1]){
            tmp_twist.linear.x = sign[1] * 0.15;
        }
        else if(angle_x < -m_angle_th[1]){
            tmp_twist.linear.x = -sign[1] * 0.15;
        }
        
    }

    //X axis
    if(m_axis[2] >= 0){
        //takes the correct angle from r,p,y indicated previously in m_axis vector
        double angle_y = angles[m_axis[2]];
        if(angle_y > m_angle_th[2]){
            tmp_twist.linear.y = sign[2] * 0.15;
        }
        else if(angle_y < -m_angle_th[2]){
            tmp_twist.linear.y = -sign[2] * 0.15;
        }
    }

    twist_msg = tmp_twist;
}

/**
 * Reset TWIST message
 */
void myo::MyoController::reset_msg(){
    //Manage command message
    command_msg.header.stamp = ros::Time::now();
    
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;

    command_msg.command = command_msg.TWIST;
    command_msg.twist = twist_msg;

    //Publish command message
    pub_command.publish(command_msg);
}


/**
 * Service to reset Myo
 * @param req request
 * @param res response
 * @return True if Myo has reset its initial pose, False otherwise
 */
bool myo::MyoController::resetMyoService(myo::ResetMyo::Request &req, myo::ResetMyo::Response &res){
    if(!req.reset){
        ROS_ERROR_STREAM("ERROR: REQUEST_RESET IS FALSE!");
        res.ready = false;
        return res.ready;
    }
    std::cout << "Press ENTER to stop and reset Myo" << std::endl;
    std::cin.ignore();
    follow = 0;
    ROS_INFO_STREAM("Publishing stopped");
    for(int t = 0; t < 100; t++){
        reset_msg();
        ros::Duration(0.01).sleep();
    }
    std::cout << "Press ENTER to restart Myo" << std::endl;
    std::cin.ignore();
    //Restart publishing
    follow = 1;
    ROS_INFO_STREAM("Publishing waiting first message");

    res.ready = true;
    return res.ready;
}