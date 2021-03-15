#coding=utf-8

import rospy
import numpy as np
import math
import sys
from geometry_msgs.msg import PoseStamped, Pose
from grasp.srv import GraspRank, GraspRankResponse

class Grasp:
    """
    Grasp class \n
    """
    def __init__(self):
        self._goal_center = None
        self._pose_ee = None
        self._rank = None
        self._service_grasp = rospy.Service("grasp_srv", GraspRank, self.handleGrasp)


    def handleGrasp(self, request):
        """
        Service that compute grasping points rank \n
        Args:
            request: request of service
        Return: response
        """
        self._goal_center = request.goal.center
        self._pose_ee = request.ee_pose
        self.computeRank()
        grasping_points = list()
        for grasp_point in self._rank[1]:
            grasping_points.append(grasp_point)
        return GraspRankResponse(grasping_points)


    def createGraspingPoint(self):
        """
        Create Grasping Points for cube \n
        Return: grasping points
            position: list of numpy array of position
            orientation: list of numpy array of orientation
        """
        #Cube dimension
        height = width = length = 0.1

        #List of position and orientation
        position = list()
        orientation = list()

        #polar coordinate
        rho = 0.17 #0.061
        theta = 0
        #num points for each cube face
        num_point = 3
        
        #Center coordinate
        x_c = self._goal_center.position.x
        y_c = self._goal_center.position.y
        z_c = self._goal_center.position.z

        #Compute Grasping Points
        #Top obj
        point_p = np.array([x_c, y_c, z_c + rho])#rho + 0.009]) #+ 0.035 in x,y
        position.append(point_p)
        point_o = self.setOrientation(0)
        orientation.append(point_o)

        '''
        #Other points
        for h in range(num_point):
            index_p = 1
            while(theta < 2*math.pi):
                x = self._goal_center.position.x + rho * math.cos(theta)
                y = self._goal_center.position.y + rho * math.sin(theta)
                z = self._goal_center.position.z + (height/2)*(float(h)/float(num_point))
                point_p = np.array([x, y, z])
                point_o = self.setOrientation(index_p)
                
                position.append(point_p)
                orientation.append(point_o)

                #Updates
                theta += math.pi/2
                index_p += 1
                
            theta = 0
        '''

        return position, orientation

    
    def setOrientation(self, value):
        """
        Set orientation from default value \n
        Args:
            value: number of cube face
        Return: quaternion orientation of point
        """
        #Set orientation
        if(value == 0): #Top


            #orientation = np.array([0.7177, 0.0104, -0.6957, 0.0258])
            
            orientation = np.array([0.515299, 0.478671, -0.50345, 0.501875])

            #orientation = np.array([0.702307, -0.110643, -0.69849, -0.0814571])
            #orientation = np.array([-0.719112, 0.0213112, 0.694439, 0.0133605])
        
        elif(value == 1): #Front
            orientation = np.array([0.00928103, -0.999851, -0.0130359, 0.00649344])

        elif(value == 2): #Right side
            orientation = np.array([0.697931, -0.716051, -0.00507108, 0.0117258])

        elif(value == 3): #Behind
            orientation = np.array([-0.00282146, 0.0156438, 0.0843281, 0.996311])

        elif(value == 4): #Left side
            orientation = np.array([0.00628831, 0.0144797, 0.718782, 0.695056])

        else:
            rospy.logerr("Index not valid!")
            rospy.on_shutdown(self.reason)

        return orientation

    
    def computeDistance(self, from_ee, to_point):
        """
        Compute distance from actual ee to grasp point \n
        Args:
            from_ee: ee position/orientation
            to_point: grasp point position/orientation
        Return: distance from ee to grasp point
        """
        from_ee = np.array(from_ee)
        to_point = np.array(to_point)
        distance = np.linalg.norm(from_ee - to_point)
        return distance


    def computeScore(self, point_p, point_o, ee_p, ee_o):
        """
        Compute score of a grasping point p \n
        Args: all args are numpy vector
            point_p: position of point
            point_o: orientation of point
            ee_p: ee position
            ee_o: ee orientation
        Return: score of point p
        """
        dist_p = self.computeDistance(ee_p, point_p)
        dist_o = self.computeDistance(ee_o, point_o)

        score = (1./dist_p) + (1./dist_o)

        return score

    
    def computeRank(self):
        """
        Use score function to compute rank of the grasping points \n
        """
        #Create grasping points
        position, orientation = self.createGraspingPoint()
        num_points = len(position)

        ee_position = np.array([self._pose_ee.position.x, self._pose_ee.position.y, self._pose_ee.position.z])
        ee_orientation = np.array([self._pose_ee.orientation.x, self._pose_ee.orientation.y, self._pose_ee.orientation.z, self._pose_ee.orientation.w])

        #Compute rank
        disorder_rank = list()
        grasping_list = list()
        for i in range(num_points):
            score = self.computeScore(position[i], orientation[i], ee_position, ee_orientation)
            disorder_rank.append(score)
            pose_msg = self.createPoseStampedMsg(i, position[i], orientation[i])
            grasping_list.append(pose_msg)
            
        #Create list(score, pose)
        list_score = zip(disorder_rank, grasping_list)
        list_score.sort(reverse=True)  
        #Ordered list  
        self._rank = zip(*list_score)
        

    def createPoseStampedMsg(self, id_p, position, orientation):
        """
        Create PoseStamped message \n
        Args:
            id_p: point ID
            position: position numpy array
            orientation: orientation numpy array
        Return: PoseStamped message
        """
        pose_msg = PoseStamped()
        pose_msg.header.seq = id_p
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1] 
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        return pose_msg


    def getRank(self):
        """
        Get rank \n
        Return rank
        """
        return self._rank


    def getGoalCenter(self):
        """
        Get center of the goal \n
        Return center
        """
        return self._goal_center


    def getEEPose(self):
        """
        Get EE pose \n
        Return: EE pose
        """
        return self._pose_ee


    def reason(self):
        rospy.logerr("Error: Index not valid! Wait to shutdown!")

