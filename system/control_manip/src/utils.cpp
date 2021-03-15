#include <control_manip/utils.h>

/**
 * Compute distance between two points
 * @param a point
 * @param b point
 * @return distance from a to b
 */
double control_manip::Utils::computeDistance(geometry_msgs::Pose a, geometry_msgs::Pose b){
    double x_a = a.position.x;
    double y_a = a.position.y;
    double z_a = a.position.z;
    double x_b = b.position.x;
    double y_b = b.position.y;
    double z_b = b.position.z;
    double distance = sqrt(pow(x_b - x_a, 2) + pow(y_b - y_a, 2) + pow(z_b - z_a, 2));
    return distance;
}

/**
 * Create geometry_msgs Pose
 * @param (x,y,z) position
 * @param (ox, oy, oz, ow) orientation
 * @return Pose msg
 */
geometry_msgs::Pose control_manip::Utils::createPose(double x, double y, double z, double ox, double oy, double oz, double ow){
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = x;
    pose_msg.position.y = y;
    pose_msg.position.z = z;
    pose_msg.orientation.x = ox;
    pose_msg.orientation.y = oy;
    pose_msg.orientation.z = oz;
    pose_msg.orientation.w = ow;
    return pose_msg;
}

/**
 * Compare two number
 * @param a
 * @param b
 * @param difference threshold difference
 * @return true if abs(a-b) < difference, false otherwise
 */
bool control_manip::Utils::compare(double a, double b, double difference){
    if(std::fabs(a - b) > difference){
        return false;
    }
    return true;
}


/**
 * Create Goal message
 * @param id id of goal
 * @param center center of the goal
 * @return Goal msg
 */
control_manip::Goal control_manip::Utils::createGoalMsg(int id, geometry_msgs::Pose center, std::vector<geometry_msgs::PoseStamped> grasping_points){
    Goal goal_msg;
    goal_msg.id = id;
    goal_msg.center = center;
    goal_msg.grasping_points = grasping_points;
    return goal_msg;
}

/**
 * Create GoalArray message
 * @param goals array of goal
 * @return GoalArray msg
 */
control_manip::GoalArray control_manip::Utils::createGoalArrayMsg(std::vector<Goal> goals){
    GoalArray goal_array_msg;
    goal_array_msg.goal = goals;
    return goal_array_msg;
}

/**
 * Create Objects message
 * @param goals array of goals
 * @param obstacles array of obstacles
 * @param joints current joints value
 * @param escape_points array of escape points
 * @return Objects msg
 */
control_manip::Objects control_manip::Utils::createObjectsMsg(std::vector<Goal> goals, std::vector<geometry_msgs::PoseArray> obstacles, std::vector<double> joints, std::vector<geometry_msgs::PoseArray> escape_points){
    Objects objects_msg;
    objects_msg.goals = createGoalArrayMsg(goals);
    //Add obstacles points
    for(int i = 0; i < obstacles.size(); i++){
        for(int j = 0; j < obstacles.at(i).poses.size(); j++){
            objects_msg.obstacles.poses.push_back(obstacles.at(i).poses.at(j));
        }
    }
    //Add joints value
    for(int i = 0; i < joints.size(); i++){
        objects_msg.joints.positions.push_back(joints.at(i));
    }
    //Add escape points
    for(int i = 0; i < escape_points.size(); i++){
        for(int j = 0; j < escape_points.at(i).poses.size(); j++){
            objects_msg.escape_points.poses.push_back(escape_points.at(i).poses.at(j));
        }
    }

    return objects_msg;
}


