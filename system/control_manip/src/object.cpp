#include <control_manip/object.h>

/**
 * Costructor of Object
 * @param apriltagDetectedPose pose object from apriltag
 */
control_manip::Object::Object(int obj_id, geometry_msgs::PoseStamped apriltagDetectedPose){
    //set ID, type, dimensions and name
    id = obj_id;
    std::stringstream stream;
    if(id >= 0 && id < 4){
        stream << "box_" << id;
        type = 0;
        height = width = length = 0.1;
    }
    else if(id >= 4 && id < 6){
        stream << "cylinder_" << id;
        type = 1;
        height = 0.50; //0.2;
        radius = 0.05;
    }
    else if(id >= 6){
        stream << "cubeObs_" << id;
        type = 2;
        height = width = length = 0.1;
    }
    name = stream.str();

    configure(apriltagDetectedPose);
}


/**
 * Configure object's center pose
 * @param apriltagDetectedPose input pose from apriltag
 */
void control_manip::Object::configure(geometry_msgs::PoseStamped apriltagDetectedPose){
    geometry_msgs::PoseStamped converted = transform(apriltagDetectedPose, WORLD_FRAME);
    setCenter(converted);
    if(id < 4){
        //std::cout << "Target with ID: " << id <<std::endl;
        createTargetCollisionObject(WORLD_FRAME);
        createGraspingPoints(WORLD_FRAME);
        
    }
    else{
        //std::cout << "Obstacle with ID: " << id <<std::endl;
        createObstacleCollisionObject(WORLD_FRAME);
        createEscapePoints();
    }
}


/**
 * Transform pose from input_frame to output_frame
 * @param input_pose
 * @param out_frame
 * @return output_pose
 */
geometry_msgs::PoseStamped control_manip::Object::transform(geometry_msgs::PoseStamped input_pose, std::string out_frame){
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    geometry_msgs::TransformStamped aprilPose_to_world;
    aprilPose_to_world = tf_buffer.lookupTransform(WORLD_FRAME, "camera_rgb_optical_frame", ros::Time(0), ros::Duration(5.0));
    geometry_msgs::PoseStamped output_pose;
    tf2::doTransform(input_pose, output_pose, aprilPose_to_world);
    return output_pose;
}


/**
 * Set object center
 * @param pose pose of object
 */
void control_manip::Object::setCenter(geometry_msgs::PoseStamped pose){
    center.position.x = pose.pose.position.x - 0.005;
    center.position.y = pose.pose.position.y;
    if(id <= 3){
        center.position.z = pose.pose.position.z;// + height;
    }
    else if(id >= 4 && id < 6){
        center.position.z = pose.pose.position.z - 0.20;// - height/2; //+ 0.19
    }
    else{
        center.position.z = pose.pose.position.z + 0.005; // + height;// - height/2;
    }
    center.orientation = pose.pose.orientation;
}


/**
 * Create collision object for target obj
 * @param frame_id frame id
 */
void control_manip::Object::createTargetCollisionObject(std::string frame_id){
    //extract roll, pith, yaw
    double roll, pitch, yaw;
    tf::Quaternion qa(center.orientation.x,
                        center.orientation.y,
                        center.orientation.z,
                        center.orientation.w);
    tf::Matrix3x3(qa).getRPY(roll, pitch, yaw);
    my_yaw = yaw;

    //create collision object
    collision.header.frame_id = frame_id;
    collision.id = name;

    geometry_msgs::Pose collisionPose;
    shape_msgs::SolidPrimitive primitive;
    
    //BOX
    if(type == 0){
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = length; //x
        primitive.dimensions[1] = width;  //y
        primitive.dimensions[2] = height; //z

        //prepare pose collision
        qa = tf::createQuaternionFromRPY(0, 0, my_yaw);

        //set pose collision
        quaternionTFToMsg(qa,collisionPose.orientation);
        collisionPose.position = center.position;
    }
    else{
        ROS_ERROR("ERROR: WRONG TYPE'S OBJECT!");
    }

    collision.primitives.push_back(primitive);
    collision.primitive_poses.push_back(collisionPose);
    collision.operation = collision.ADD;
}


/**
 * Get collision object
 * @return collision msgs
 */
moveit_msgs::CollisionObject control_manip::Object::getCollisionObject(){
    return collision;
}


/**
 * Create grasping point of a object
 * @param frame_id frame id of the object
 */
void control_manip::Object::createGraspingPoints(std::string frame_id){ 
    //grasping point
    geometry_msgs::PoseStamped grasp_point;
    grasp_point.header.frame_id = frame_id;
    //polar coordinate
    double rho = 0.14; //0.061;
    double theta = 0;
    int it = 0;

    //grasping points for Cube and Cylinder
    if(type == 0 || type == 1){
        //first point (top of the object)
        grasp_point.header.seq = it;
        grasp_point.pose.position.x = center.position.x;
        grasp_point.pose.position.y = center.position.y;
        grasp_point.pose.position.z = center.position.z + rho;
        if(type == 1){
            grasp_point.pose.position.z += 0.05;
        }
        grasp_point.pose.orientation = setQuaternion(0); 
        grasping_points_list.push_back(grasp_point);
        //for visualize it in rviz
        marker_point.points.push_back(grasp_point.pose.position);

        /*
        int num_points = 3;
        it++;

        //other points
        for(double h = 0; h < num_points; h++){
            int index_p = 1;
            while(theta < 2*M_PI){
                grasp_point.header.seq = it;
                grasp_point.pose.position.x = center.position.x + rho*cos(theta);
                grasp_point.pose.position.y = center.position.y + rho*sin(theta);
                grasp_point.pose.position.z = center.position.z + (height/2)*(h/num_points);
                grasp_point.pose.orientation = setQuaternion(index_p);
                grasping_points_list.push_back(grasp_point);
                //for visualize it in rviz
                marker_point.points.push_back(grasp_point.pose.position);

                //updates
                theta += M_PI/2;
                it++;
                index_p++;
            }
            theta = 0;   
        }
        */
    
    }//if

}


/**
 * Set quaternion of a grasping point
 * @param index index of a cube face
 * @return Quaternion msgs of grasping point
 */
geometry_msgs::Quaternion control_manip::Object::setQuaternion(int index){
    geometry_msgs::Quaternion q;
    
    switch(index){
        case 0:
            q.x = 0.515299;
            q.y = 0.478671;
            q.z = -0.50345;
            q.w = 0.501875;
            break;
        
        case 1:
            q.x = 0.00928103;
            q.y = -0.999851;
            q.z = -0.0130359;
            q.w = 0.00649344;
            break;

        case 2:
            q.x = 0.697931;
            q.y = -0.716051;
            q.z = -0.00507108;
            q.w = 0.0117258;
            break;

        case 3:
            q.x = -0.00282146;
            q.y = 0.0156438;
            q.z = 0.0843281;
            q.w = 0.996311;
            break;
    
        case 4:
            q.x = 0.00628831;
            q.y = 0.0144797;
            q.z = 0.718782;
            q.w = 0.695056;
            break;

        default:
            ROS_ERROR("Index not valid!");
    }
    return q;

}


/**
 * Get grasping points
 * @return vector of grasping points
 */
std::vector<geometry_msgs::PoseStamped> control_manip::Object::getGraspingPoints(){
    return grasping_points_list;
}


/**
 * Create a visualization msgs to visualize grasping points
 * @return visualization msgs Marker of grasping points
 */
visualization_msgs::Marker control_manip::Object::visualizeGraspingPoints(){
    marker_point.header.frame_id = "world";
    marker_point.header.stamp = ros::Time::now();
    marker_point.ns = "Grasp points";
    marker_point.action = visualization_msgs::Marker::ADD;
    marker_point.id = 0;
    marker_point.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_point.color.g = 1.0f;
    marker_point.color.a = 1.0;
    marker_point.scale.x = 0.01;
    marker_point.scale.y = 0.01;
    marker_point.scale.z = 0.01;

    return marker_point;
}


/**
 * Create points that represent obstacle
 */
void control_manip::Object::createObstacle(){
    double rho;
    if(id >= 6){
        rho = length/2;
    }
    else{
        rho = radius;
    }
    double d = sqrt(2)*rho;
    double theta = 0;
    double beta = asin(rho/d);

    geometry_msgs::Pose obs2;
    //first point
    obs2.position = center.position;
    obs2.orientation = center.orientation;
    obstacle.poses.push_back(obs2);

    
    const int LIMIT_NUMBER = 20;
    for(double h = 0; h <= LIMIT_NUMBER; h++){
        while(theta < 2*M_PI){
            //Cube obstacle
            if(id >= 6){
                //side face
                obs2.position.x = center.position.x + rho*cos(theta);
                obs2.position.y = center.position.y + rho*sin(theta);
                obs2.position.z = center.position.z + (height/2)*(h/LIMIT_NUMBER);
                obs2.orientation = center.orientation;
                obstacle.poses.push_back(obs2);
                //corners
                obs2.position.x = center.position.x + d*cos(beta);
                obs2.position.y = center.position.y + d*sin(beta);
                obs2.position.z = center.position.z + (height/2)*(h/LIMIT_NUMBER);
                obs2.orientation = center.orientation;
                obstacle.poses.push_back(obs2);

                if(h != 0){
                    //side face
                    obs2.position.x = center.position.x + rho*cos(theta);
                    obs2.position.y = center.position.y + rho*sin(theta);
                    obs2.position.z = center.position.z - (height/2)*(h/LIMIT_NUMBER);
                    obs2.orientation = center.orientation;
                    obstacle.poses.push_back(obs2);
                    //corners
                    obs2.position.x = center.position.x + d*cos(beta);
                    obs2.position.y = center.position.y + d*sin(beta);
                    obs2.position.z = center.position.z - (height/2)*(h/LIMIT_NUMBER);
                    obs2.orientation = center.orientation;
                    obstacle.poses.push_back(obs2);
                }
                theta += M_PI/2;
                beta += M_PI/2;
            }
            //Prism obstacle
            else{
                obs2.position.x = center.position.x + rho*cos(theta);
                obs2.position.y = center.position.y + rho*sin(theta);
                obs2.position.z = center.position.z + (height/2)*(h/LIMIT_NUMBER);
                obs2.orientation = center.orientation;
                obstacle.poses.push_back(obs2);

                if(h != 0){
                    obs2.position.x = center.position.x + rho*cos(theta);
                    obs2.position.y = center.position.y + rho*sin(theta);
                    obs2.position.z = center.position.z - (height/2)*(h/LIMIT_NUMBER);
                    obs2.orientation = center.orientation;
                    if(obs2.position.z >= 0.95){
                        obstacle.poses.push_back(obs2);
                    }
                }
                theta += M_PI/6;
            }
        }
        theta = 0;
    }
    

}


/**
 * Get obstacle
 * @return PoseArray of points that model obstacle
 */
geometry_msgs::PoseArray control_manip::Object::getObstacle(){
    return obstacle;
}


/**
 * Create collision object of the obstacle
 * @param frame_id frame_id
 */
void control_manip::Object::createObstacleCollisionObject(std::string frame_id){
    //First of all create obstacle
    createObstacle();

    //Now create collision object for the obstacle
    collision.header.frame_id = frame_id;
    collision.id = name;
    for(int i = 0; i < obstacle.poses.size(); i++){
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = 0.005;
        collision.primitives.push_back(primitive);
        collision.primitive_poses.push_back(obstacle.poses.at(i));
    }
    collision.operation = collision.ADD;
}


/**
 * Create escape points
 */
void control_manip::Object::createEscapePoints(){
    double rho = 0.15; //0.11; //0.09;
    double d = sqrt(2)*rho;
    double beta = asin(rho/d);
    double theta = 0;

    /*
    geometry_msgs::Pose point;
    for(int i = 0; i < 4; i++){
        point.position.x = center.position.x + d*cos(beta);
        point.position.y = center.position.y + d*sin(beta);
        point.position.z = center.position.z + d;
        point.orientation = center.orientation;
        escape_points.poses.push_back(point);

        beta += M_PI/2;
    }
    */

    geometry_msgs::Pose obs2;

    //first point
    obs2.position = center.position;
    obs2.position.z += height/2 + 0.10;
    obs2.orientation = center.orientation;
    escape_points.poses.push_back(obs2);

    const int LIMIT_NUMBER = 2;
    for(double h = 0; h <= LIMIT_NUMBER; h++){
        while(theta < 2*M_PI){
            //Cube obstacle
            if(id >= 6){
                //side face
                obs2.position.x = center.position.x + rho*cos(theta);
                obs2.position.y = center.position.y + rho*sin(theta);
                obs2.position.z = center.position.z + (height/2)*(h/LIMIT_NUMBER);
                obs2.orientation = center.orientation;
                escape_points.poses.push_back(obs2);
                //corners
                obs2.position.x = center.position.x + d*cos(beta);
                obs2.position.y = center.position.y + d*sin(beta);
                obs2.position.z = center.position.z + (height/2)*(h/LIMIT_NUMBER);
                obs2.orientation = center.orientation;
                escape_points.poses.push_back(obs2);

                /*
                if(h != 0){
                    //side face
                    obs2.position.x = center.position.x + rho*cos(theta);
                    obs2.position.y = center.position.y + rho*sin(theta);
                    obs2.position.z = center.position.z - (height/2)*(h/LIMIT_NUMBER);
                    obs2.orientation = center.orientation;
                    escape_points.poses.push_back(obs2);
                    //corners
                    obs2.position.x = center.position.x + d*cos(beta);
                    obs2.position.y = center.position.y + d*sin(beta);
                    obs2.position.z = center.position.z - (height/2)*(h/LIMIT_NUMBER);
                    obs2.orientation = center.orientation;
                    escape_points.poses.push_back(obs2);
                }
                */
                theta += M_PI/2;
                beta += M_PI/2;
            }
            //Prism obstacle
            else{
                obs2.position.x = center.position.x + rho*cos(theta);
                obs2.position.y = center.position.y + rho*sin(theta);
                obs2.position.z = center.position.z + (height/2)*(h/LIMIT_NUMBER) + 0.03;
                obs2.orientation = center.orientation;
                if(obs2.position.z >= 0.95){
                    escape_points.poses.push_back(obs2);
                }
                

                /*
                if(h != 0){
                    obs2.position.x = center.position.x + rho*cos(theta);
                    obs2.position.y = center.position.y + rho*sin(theta);
                    obs2.position.z = center.position.z - (height/2)*(h/LIMIT_NUMBER);
                    obs2.orientation = center.orientation;
                    if(obs2.position.z >= 0.90){
                        escape_points.poses.push_back(obs2);
                    }
                }
                */
                theta += M_PI/3;
            }
        }
        theta = 0;
    }



}


/**
 * Get escpae points
 * @return escape points for the obstacle
 */
geometry_msgs::PoseArray control_manip::Object::getEscapePoints(){
    return escape_points;
}


/**
 * Get name of object
 * @return object's name
 */
std::string control_manip::Object::getName(){
    return name;
}


/**
 * Get object center
 * @return object's center
 */
geometry_msgs::Pose control_manip::Object::getCenter(){
    return center;
}


/**
 * Get type of object
 * @return object's type
 */
int control_manip::Object::getType(){
    return type;
}


/**
 * Get object ID
 * @return object's ID
 */
int control_manip::Object::getId(){
    return id;
}


/**
 * Get dimension of object
 * @return object's dimension: (height, radius) if cylinder, (height, width, length) if cube
 */
std::vector<double> control_manip::Object::getDimension(){
    std::vector<double> dimensions;
    if(id >= 4 && id < 6){
        dimensions = {height, radius};
    }
    else{
        dimensions = {height, width, length};
    }

    return dimensions;
}



void control_manip::Object::createSupport(){
    double rho;
    if(id >= 6){
        rho = length/2;
    }
    else{
        rho = radius;
    }
    double d = sqrt(2)*rho;
    double theta = 0;
    double beta = asin(rho/d);

    geometry_msgs::Pose obs3;
    geometry_msgs::Pose obs2;
    obs3.position = center.position;
    obs3.position.z -= (0.09 + height/2); 
    obs3.orientation = center.orientation;

    for(double h = 0; h <= 4; h++){
        while(theta < 2*M_PI){
            //Cube obstacle
            if(id >= 6){
                //side face
                obs2.position.x = obs3.position.x + rho*cos(theta);
                obs2.position.y = obs3.position.y + rho*sin(theta);
                obs2.position.z = obs3.position.z + (height/2)*(h/4);
                obs2.orientation = obs3.orientation;
                support_points.poses.push_back(obs2);
                //corners
                obs2.position.x = obs3.position.x + d*cos(beta);
                obs2.position.y = obs3.position.y + d*sin(beta);
                obs2.position.z = obs3.position.z + (height/2)*(h/4);
                obs2.orientation = obs3.orientation;
                support_points.poses.push_back(obs2);

                if(h != 0){
                    //side face
                    obs2.position.x = obs3.position.x + rho*cos(theta);
                    obs2.position.y = obs3.position.y + rho*sin(theta);
                    obs2.position.z = obs3.position.z - (height/2)*(h/4);
                    obs2.orientation = obs3.orientation;
                    support_points.poses.push_back(obs2);
                    //corners
                    obs2.position.x = obs3.position.x + d*cos(beta);
                    obs2.position.y = obs3.position.y + d*sin(beta);
                    obs2.position.z = obs3.position.z - (height/2)*(h/4);
                    obs2.orientation = obs3.orientation;
                    support_points.poses.push_back(obs2);
                }
                theta += M_PI/2;
                beta += M_PI/2;
            }
            //Prism obstacle
            else{
                obs2.position.x = obs3.position.x + rho*cos(theta);
                obs2.position.y = obs3.position.y + rho*sin(theta);
                obs2.position.z = obs3.position.z + (height/2)*(h/4);
                obs2.orientation = obs3.orientation;
                support_points.poses.push_back(obs2);

                if(h != 0){
                    obs2.position.x = obs3.position.x + rho*cos(theta);
                    obs2.position.y = obs3.position.y + rho*sin(theta);
                    obs2.position.z = obs3.position.z - (height/2)*(h/4);
                    obs2.orientation = obs3.orientation;
                    support_points.poses.push_back(obs2);
                }
                theta += M_PI/6;
            }
        }
        theta = 0;
    }
}


geometry_msgs::PoseArray control_manip::Object::getSupport(){
    return support_points;
}

