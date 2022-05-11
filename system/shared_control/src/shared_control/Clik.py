#coding=utf-8

import numpy as np
import Utils
import copy

class Clik:
    """
    Closed Loop Inverse Kinematics algorithm \n
    Args:
        K: definite positive matrix
        delta_t: interval time
        rob_kin: RobotKinematics object
        init_q: initial joints vector
    """
    def __init__(self, K, delta_t, rob_kin, init_q):
        self._K = K
        self._delta_t = delta_t
        self._rob_kin = rob_kin
        self._init_q = init_q
        self._q_clik = np.zeros(6)
        self._ee_pose_clik = np.zeros((4,4))
        self._indexEE = len(init_q)

        self._limit_z = 0.90
        
    def computeCLIK(self, actual_position, twist_ee, jacobian, new_q):
        """
        Use CLIK to compute new joints vector q and ee pose \n
        Args:
            actual_position: actual EE position (x,y,z)
            twist_ee: twist for EE
            jacobian: jacobian matrix
        Return:
            new joints vector q
            new pose of EE
            final twist used to compute new joints vector and ee pose
        """
        self._init_q = new_q
        twist_f = np.zeros(6)
        done = self.doCLIK(actual_position, twist_ee, jacobian)
        if(not done):
            tw_ee = copy.copy(twist_ee)
            tw_ee[2] = 0
            print("New FINAL TWIST: " +str(tw_ee))
            done = self.doCLIK(actual_position, tw_ee, jacobian) 
            twist_f = tw_ee

        return self._q_clik, self._ee_pose_clik, twist_f

    def doCLIK(self, actual_position, twist_ee, jacobian):
        """
        CLIK computation \n
        Args:
            actual_position: actual EE position (x,y,z)
            twist_ee: twist for EE
            jacobian: jacobian matrix
        Return: True if computation is done, False otherwise
        """
        #compute inverse jacobian
        jacob_inv = self._rob_kin.computeInverseJacobian(jacobian)
        #compute vel joints
        vel_joints = np.dot(jacob_inv, twist_ee)
        #compute new joints angles q
        new_q = self._init_q + vel_joints * self._delta_t
        #compute error e = x_d - x_e
        x_e = self._rob_kin.getPosition(new_q, self._indexEE)
        x_d = self.nextPoint(twist_ee, actual_position)
        err = Utils.computeError(x_e, x_d)
        #compute new joints velocities and angles
        st = twist_ee + np.dot(self._K, err)
        vel_joints_clik = np.dot(jacob_inv, st)
        #Euler Integration
        self._q_clik = self._init_q + vel_joints_clik * self._delta_t 
        #Matrix EE pose
        self._ee_pose_clik = self._rob_kin.get_pose(self._q_clik, self._indexEE)  

        #Check the limit in Z
        if(self._ee_pose_clik[2,3] <= self._limit_z):
            print("LIMITE IN Z")
            return False
    
        #Update
        self._init_q = self._q_clik
        return True

    def getQClik(self):
        """
        Get joints vector q from CLIK \n
        Return: joints vector q
        """
        return copy.copy(self._q_clik)

    def getEEPoseClik(self):
        """
        Get EE pose from CLIK \n
        Return: end effector pose
        """
        return copy.copy(self._ee_pose_clik)

    def nextPoint(self, twist, actual_position):
        """
        Compute next position in ideal trajectory \n
        Args:
            twist_user: End_Effector input velocity
        Return: next position
        """
        d = twist[0:3] * self._delta_t 
        #compute next position
        new_position = actual_position + d
        return new_position


