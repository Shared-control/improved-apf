#include <control_manip/manager.h>

/**
 * Costructor of Manager class
 * @param t_params Input Parameters
 * @param t_nh_ptr NodeHandle Ptr
 * @param t_manipulation Manipulation
 */
control_manip::Manager::Manager(InputParameters t_params, 
                                std::shared_ptr<ros::NodeHandle> t_nh_ptr,
                                control_manip::Manipulation t_manipulation)
    : m_params(t_params)
    , m_nh_ptr(t_nh_ptr)
    , m_manipulation(t_manipulation)
{}


/**
 * Start manager: start service and go home pose
 */
void control_manip::Manager::start(){
    std::cout << "Go home!" << std::endl;
    if(!m_manipulation.goHome()){
        ROS_ERROR_STREAM("Error: movement to HOME failed");
        ros::shutdown();
    }

    //If dynamic system use msg to manage objects
    if(m_params.dynamic){
        m_pub_objs = m_nh_ptr->advertise<control_manip::Objects>("objects_msg", 100);
        m_sub_april = m_nh_ptr->subscribe("/tag_detections", 1, &control_manip::Manager::callbackApriltag, this);
    }
    //if static use srv
    else{
        std::cout << "Press ENTER to start system" <<std::endl;
        std::cin.ignore();
        m_objects_server = m_nh_ptr->advertiseService("/objects_srv", &control_manip::Manager::objService, this);
        ROS_WARN_STREAM("DONE");
    }
}


/**
 * Set Objects message
 */
void control_manip::Manager::setObjectsParameters(){
    //Get current home joints value
    std::vector<double> joints = m_manipulation.getCurrentJoint();

    //Create Objects msg
    std::vector<control_manip::Goal> goal_vector;
    std::vector<geometry_msgs::PoseArray> obstacles;
    std::vector<geometry_msgs::PoseArray> escape_points;
    std::vector<moveit_msgs::CollisionObject> collision_object_vector;
    std::vector<geometry_msgs::PoseStamped> grasping_points;

    for(int i = 0; i < m_msg_apriltag_ptr->detections.size(); i++){
        geometry_msgs::PoseStamped stamped_msg;
        int id = m_msg_apriltag_ptr->detections[i].id.at(0);
        stamped_msg.pose = m_msg_apriltag_ptr->detections[i].pose.pose.pose;
        control_manip::Object obj(id, stamped_msg);
        collision_object_vector.push_back(obj.getCollisionObject());
        
        //If current object is a goal
        int id_obj = obj.getId();
        if(id_obj < 4){            
            control_manip::Goal goal_msg = control_manip::Utils::createGoalMsg(id_obj, obj.getCenter(), obj.getGraspingPoints());
            goal_vector.push_back(goal_msg);
            //Since I create only one grasping point
            grasping_points.push_back(obj.getGraspingPoints().at(0));
        }
        //If it is obstacle
        else{
            obstacles.push_back(obj.getObstacle());
            escape_points.push_back(obj.getEscapePoints());
        }
    }
    //Create wall for user kinect
    moveit_msgs::CollisionObject wall = createWall();
    collision_object_vector.push_back(wall);

    //Add collision objects to scene
    m_manipulation.updateCollisionScene(collision_object_vector);

    //Show escape points and grasping points in the scene
    m_manipulation.visualizeMarkerEscape(escape_points);
    m_manipulation.visualizeMarkerGrasping(grasping_points);

    //Send informations to Kinematics node: targets, obstacles, escape points and initial joints values
    m_objects_msg = control_manip::Utils::createObjectsMsg(goal_vector, obstacles, joints, escape_points);

}


/**
 * Apriltag callback for dynamic system
 * @param msg_april aprilatgDetectionArray message
 */
void control_manip::Manager::callbackApriltag(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg_april){
    m_msg_apriltag_ptr = msg_april;
    //createCollisionForRviz();
    m_pub_objs.publish(m_objects_msg);
}


/**
 * Service objects for static system
 * @param req request
 * @param res response
 */ 
bool control_manip::Manager::objService(control_manip::InitObj::Request &req, control_manip::InitObj::Response &res){
    if(!req.status.ready){
        ROS_ERROR_STREAM("Error!");
        ros::shutdown();
    }
    //Read objects from apriltag detector node
    m_msg_apriltag_ptr = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
    if(m_msg_apriltag_ptr == NULL){
        ROS_ERROR_STREAM("Something was wrong!");
        ros::shutdown();
    }
    //set Objects message
    setObjectsParameters();

    //set response with Objects message
    res.objects = m_objects_msg;
    return true;
}


void control_manip::Manager::createCollisionForRviz(){
    //Get current home joints value
    std::vector<double> joints = m_manipulation.getCurrentJoint();

    //Create Objects msg
    std::vector<control_manip::Goal> goal_vector;
    std::vector<geometry_msgs::PoseArray> obstacles;
    std::vector<geometry_msgs::PoseArray> escape_points;
    std::vector<moveit_msgs::CollisionObject> collision_object_vector;
    std::vector<geometry_msgs::PoseStamped> grasping_points;

    std::vector<moveit_msgs::ObjectColor> obj_color;
    
    for(int i = 0; i < m_msg_apriltag_ptr->detections.size(); i++){
        geometry_msgs::PoseStamped stamped_msg;
        int id = m_msg_apriltag_ptr->detections[i].id.at(0);
        stamped_msg.pose = m_msg_apriltag_ptr->detections[i].pose.pose.pose;
        control_manip::Object obj(id, stamped_msg);
        std_msgs::ColorRGBA tmp_color;
        moveit_msgs::ObjectColor oc;

        //If current object is a goal
        int id_obj = obj.getId();
        if(id_obj < 4){

            collision_object_vector.push_back(obj.getCollisionObject());

            tmp_color.r = 1.0;
            tmp_color.a = 1.0;

            oc.color = tmp_color;
            obj_color.push_back(oc);

            control_manip::Goal goal_msg = control_manip::Utils::createGoalMsg(id_obj, obj.getCenter(), obj.getGraspingPoints());
            goal_vector.push_back(goal_msg);
            //Since I create only one grasping point
            grasping_points.push_back(obj.getGraspingPoints().at(0));
        }
        //If it is obstacle
        else{
            collision_object_vector.push_back(createCollisionObject(obj));
            
            //Yellow Cyl
            if(id_obj == 4 || id_obj == 5){
                tmp_color.r = 1.0;
                tmp_color.g = 1.0;
                tmp_color.b = 0.0;
                tmp_color.a = 1.0;
                oc.color = tmp_color;
                obj_color.push_back(oc);

                collision_object_vector.push_back(createSupportCollision(obj));
            
                //Support
                tmp_color.r = 0.0;
                tmp_color.g = 1.0;
                tmp_color.b = 0.0;
                tmp_color.a = 1.0;
                oc.color = tmp_color;
                obj_color.push_back(oc);

            }

            //Blue Cube
            else{
                tmp_color.b = 1.0;
                tmp_color.a = 1.0;
                oc.color = tmp_color;
                obj_color.push_back(oc);

                if(id_obj == 9 || id_obj == 12){
                    collision_object_vector.push_back(createSupportCollision(obj));
                    //Support
                    tmp_color.r = 0.0;
                    tmp_color.g = 1.0;
                    tmp_color.b = 0.0;
                    tmp_color.a = 1.0;
                    oc.color = tmp_color;
                    obj_color.push_back(oc);
                }
            }

            obstacles.push_back(obj.getObstacle());
            escape_points.push_back(obj.getEscapePoints());
        }
    }
    //Create wall for user kinect
    moveit_msgs::CollisionObject wall = createWall();
    collision_object_vector.push_back(wall);
    std_msgs::ColorRGBA tmp_color;
    tmp_color.r = 0.7;
    tmp_color.g = 0.7;
    tmp_color.b = 0.7;
    tmp_color.a = 1.0;
    moveit_msgs::ObjectColor oc;
    oc.color = tmp_color;
    obj_color.push_back(oc);

    //Add collision objects to scene
    m_manipulation.updateCollisionScene(collision_object_vector, obj_color);

    //Show escape points and grasping points in the scene
    m_manipulation.visualizeMarkerEscape(escape_points);
    m_manipulation.visualizeMarkerGrasping(grasping_points);

    //Send informations to Kinematics node: targets, obstacles, escape points and initial joints values
    m_objects_msg = control_manip::Utils::createObjectsMsg(goal_vector, obstacles, joints, escape_points);
}


moveit_msgs::CollisionObject control_manip::Manager::createCollisionObject(Object object){
    moveit_msgs::CollisionObject coll_obj;
    coll_obj.header.frame_id = "world";
    geometry_msgs::Pose collisionPose;
    shape_msgs::SolidPrimitive primitive;

    std::stringstream stream;

    int id = object.getId();
    if(id == 4 || id == 5){
        //std::cout << "Cylinder with id: " << id << std::endl;
        stream << "cyl_" << id;
        coll_obj.id = stream.str();

        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = 0.2; //Height
        primitive.dimensions[1] = 0.05; //radius
        
        collisionPose = object.getCenter();
        collisionPose.position.z += 0.1;
        tf::Quaternion qa = tf::createQuaternionFromRPY(0, 0, 0);
        tf::quaternionTFToMsg(qa, collisionPose.orientation);
    }

    if(id == 9 || id == 12 || id == 15){
        //std::cout << "obsCube with id: " << id << std::endl;
        stream << "obsCube_" << id;
        coll_obj.id = stream.str();

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;
        primitive.dimensions[1] = 0.1;
        primitive.dimensions[2] = 0.1;
        
        collisionPose = object.getCenter();
        tf::Quaternion qa = tf::createQuaternionFromRPY(0, 0, 0);
        tf::quaternionTFToMsg(qa, collisionPose.orientation);

    }

    coll_obj.primitives.push_back(primitive);
    coll_obj.primitive_poses.push_back(collisionPose);
    coll_obj.operation = coll_obj.ADD;

    return coll_obj;
}


moveit_msgs::CollisionObject control_manip::Manager::createSupportCollision(Object object){
    moveit_msgs::CollisionObject coll_obj;
    coll_obj.header.frame_id = "world";
    geometry_msgs::Pose collisionPose;
    shape_msgs::SolidPrimitive primitive;

    std::stringstream stream;

    int id = object.getId();
    if(id == 4 || id == 5 || id == 12){
        if(id == 12){
            //std::cout << "SupportObs with id: " << id << std::endl;
            stream << "obsCubeSupport_" << id;
            coll_obj.id = stream.str();

            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.12;
            primitive.dimensions[1] = 0.12;
            primitive.dimensions[2] = 0.06;
            
            collisionPose = object.getCenter();
            collisionPose.position.z -= 0.14;
            tf::Quaternion qa = tf::createQuaternionFromRPY(0, 0, 0);
            tf::quaternionTFToMsg(qa, collisionPose.orientation);
        }
        else{
            //std::cout << "SupportObs with id: " << id << std::endl;
            stream << "obsCylSupport_" << id;
            coll_obj.id = stream.str();

            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.12;
            primitive.dimensions[1] = 0.12;
            primitive.dimensions[2] = 0.06;
            
            collisionPose = object.getCenter();
            collisionPose.position.z; // -= 0.19;
            tf::Quaternion qa = tf::createQuaternionFromRPY(0, 0, 0);
            tf::quaternionTFToMsg(qa, collisionPose.orientation);
        }
    }
    else if(id == 9){
        //std::cout << "SupportObs with id: " << id << std::endl;
        stream << "obsCubeSupport_" << id;
        coll_obj.id = stream.str();

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;
        primitive.dimensions[1] = 0.1;
        primitive.dimensions[2] = 0.1;
        
        collisionPose = object.getCenter();
        collisionPose.position.z -= 0.1;
        tf::Quaternion qa = tf::createQuaternionFromRPY(0, 0, 0);
        tf::quaternionTFToMsg(qa, collisionPose.orientation);
    }

    coll_obj.primitives.push_back(primitive);
    coll_obj.primitive_poses.push_back(collisionPose);
    coll_obj.operation = coll_obj.ADD;

    return coll_obj;

}


moveit_msgs::CollisionObject control_manip::Manager::createWall(){
    moveit_msgs::CollisionObject wall;
    wall.header.frame_id = "world";
    wall.id = "wall_camera_user";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 2;

    geometry_msgs::Pose wall_pose = Utils::createPose(0, 0.40, 1.50, 0, 0, 0, 1.0);

    wall.primitives.push_back(primitive);
    wall.primitive_poses.push_back(wall_pose);
    wall.operation = wall.ADD;

    return wall;

}