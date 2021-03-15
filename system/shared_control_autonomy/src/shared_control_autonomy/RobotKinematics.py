#coding=utf-8

import time
import numpy as np 
import math
import sympy as smp 
import Utils 

class RobotKinematics:
    """
    Robot kinematics Class \n
    Denavit-Hartenberg parameters \n
    Args:
        theta (list [rad]): list of angles symbols from Denavit-Hartenberg params
        a (list [m]): Denavit-Hartenberg params 
        d (list [m]): Denavit-Hartenberg params
        alpha (list [rad]): Denavit-Hartenberg params
        gripper_active: True if there is a gripper, False otherwise
    """ 
    def __init__(self, theta, a, d, alpha, robot_type, gripper_active):
        self._theta = theta
        self._a = a
        self._d = d
        self._alpha = alpha
        self._robot_type = robot_type
        self._number_joints = len(theta)
        self._numpyHM = []
        self._numpyAAJ = []
        self._gripper_active = gripper_active

        #Compute all numpy matrices
        self.initializeNumpyMatrices()


    def initializeNumpyMatrices(self):
        """
        Function to initialize all matrices in numpy form.
        """
        #Compute Homogeneous Matrices
        print("Compute Homogeneous Matrices")
        startH = time.time()
        hom_matrices = self.computeHomogeneousMatrix()  
        #Trasform sympy matrix into numpy matrix to evaluate it faster
        for matr in hom_matrices:
            self._numpyHM.append(self.transformSyminNum(matr))
        endH = time.time()
        print("Time to compute Homogeneous matrices S+N: " + str(endH-startH))
        
        #Compute Analytical Jacobian Matrices
        #all analytical jacobian matrices Sympy form = aajSm
        print("Compute analytical jacobian matrices")
        stratA = time.time()
        #Trasform sympy matrix into numpy matrix to evaluate it faster
        aajSm = self.computeAllSymbolAnalyticalJacobian(hom_matrices)
        for i in range(len(aajSm)):
            tmp = []
            for j in range(len(aajSm[i])):
                tmp.append(self.transformSyminNum(aajSm[i][j])) 
            self._numpyAAJ.append(tmp)
        endA = time.time()
        print("Time to compute analytical jacobian matrices S+N: " + str(endA-stratA))


    def computeHomogeneousMatrix(self):
        """
        Function to compute homogeneous matrices for all joints and End-Effector in symbolic form. \n
        Return: homogeneous matrices for all joints and EE in symbolic form (sympy)
        """
        #matrices from D-H parameters
        A = []
        for i in range(self._number_joints):
            hm = smp.Matrix(4, 4, [smp.cos(self._theta[i]), -smp.sin(self._theta[i])*smp.cos(self._alpha[i]), smp.sin(self._theta[i])*smp.sin(self._alpha[i]), self._a[i]*smp.cos(self._theta[i]),
                                   smp.sin(self._theta[i]), smp.cos(self._theta[i])*smp.cos(self._alpha[i]), -smp.cos(self._theta[i])*smp.sin(self._alpha[i]), self._a[i]*smp.sin(self._theta[i]),
                                   0, smp.sin(self._alpha[i]), smp.cos(self._alpha[i]), self._d[i],
                                   0, 0, 0, 1])
            A.append(hm)
            
        #Matrix base_link -> 0
        Ab = smp.Matrix(4,4,[-1, 0, 0, 0,
                              0, -1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1])

        if((self._robot_type == "ur10") or (self._robot_type == "/ur10")):
            #Matrix World -> base_link
            '''
            A_world_base = smp.Matrix(4, 4, [0, 1, 0, 0.56,
                                            -1, 0, 0, 0.045,
                                             0, 0, 1, 0.95,    
                                             0, 0, 0, 1])
            '''
            A_world_base = smp.Matrix(4, 4, [0, -1, 0, 0,
                                             1,  0, 0, 0,
                                             0,  0, 1, 0.95,    
                                             0,  0, 0, 1])
            
            t = A_world_base * Ab
            

        elif((self._robot_type == "ur5") or (self._robot_type == "/ur5")):
            A_world_base_ur5 = smp.Matrix(4, 4, [1, 0, 0, 0,
                                                 0, 1, 0, 0,
                                                 0, 0, 1, 0.87,    
                                                 0, 0, 0, 1])

            t = A_world_base_ur5 * Ab 

        else:
            print("Error: Invalid robot name!")
            
        #Matrix 6 -> EE
        Aee = smp.Matrix(4,4, [0, -1, 0, 0,
                               0, 0, -1, 0,
                               1, 0, 0, 0,
                               0, 0, 0, 1])

        Agripper = smp.Matrix(4, 4, [0, -1,  0, 0, #-0.005,
                                     0,  0, -1, 0,
                                     1,  0,  0, 0.16, #0.16 = 0.07 + 0.09 calcolato io no < 0.09
                                     0,  0,  0, 1])
        

        #Homogeneous Transformation matrices
        T = []
        for i in range(self._number_joints):
            t = t * A[i]
            T.append(t)

        #Add EE matrix
        T.append(T[self._number_joints-1]*Aee) #T=[Tb1, Tb2, Tb3, Tb4, Tb5, Tb6, Tbee]

        if(self._gripper_active):
            #Add gripper matrix
            matr_w3_gripper = T[self._number_joints-1] * Agripper
            T.append(matr_w3_gripper) #T=[Tb1, Tb2, Tb3, Tb4, Tb5, Tb6, Tbee, Tbgripper]

        return T


    def computeAllSymbolAnalyticalJacobian(self, T):
        """
        Function to compute Analytical Jacobian matrices for all joints and EE in symbolic form (sympy). \n
        Args:
            T: list of all homogeneous matrix in symbolic form (sympy)
        Return: list of all possible Jacobian matrices for all joints and EE, in symbolic form, calculated using the Euler ZYZ convention.
        """

        final_list = []
        #Compute Jacobian for all joints and EE
        for id_joint in range(self._number_joints + 1):
            #Euler's angles ZYZ
            phi1 = smp.atan2(T[id_joint][1,2],T[id_joint][0,2]) #r13>0  r13=T[0,2], r23=T[1,2] 
            phi2 = smp.atan2(T[id_joint][1,2],-T[id_joint][0,2]) #r23>0, r13<0
            phi3 = smp.atan2(-T[id_joint][1,2],-T[id_joint][0,2]) #r23<0, r13<0
    
            theta_eul1 = smp.atan2(smp.sqrt(pow(T[id_joint][0,2],2)+pow(T[id_joint][1,2],2)), T[id_joint][2,2]) #r33>0 r33=T[2,2]
            theta_eul2 = smp.atan2(smp.sqrt(pow(T[id_joint][0,2],2)+pow(T[id_joint][1,2],2)), -T[id_joint][2,2]) #r33<0

            psi1 = smp.atan2(T[id_joint][2,1],-T[id_joint][2,0]) #r31>0  r31=T[2,0], r32=T[2,1]
            psi2 = smp.atan2(T[id_joint][2,1], T[id_joint][2,0]) #r32>0, r31<0
            psi3 = smp.atan2(-T[id_joint][2,1], T[id_joint][2,0]) #r32<0, r31<0

            #Rotation matrices for Euler ZYZ
            C111 = smp.Matrix([phi1, theta_eul1, psi1])
            C112 = smp.Matrix([phi1, theta_eul1, psi2])
            C113 = smp.Matrix([phi1, theta_eul1, psi3])
            C121 = smp.Matrix([phi1, theta_eul2, psi1])
            C122 = smp.Matrix([phi1, theta_eul2, psi2])
            C123 = smp.Matrix([phi1, theta_eul2, psi3])

            C211 = smp.Matrix([phi2, theta_eul1, psi1])
            C212 = smp.Matrix([phi2, theta_eul1, psi2])
            C213 = smp.Matrix([phi2, theta_eul1, psi3])
            C221 = smp.Matrix([phi2, theta_eul2, psi1])
            C222 = smp.Matrix([phi2, theta_eul2, psi2])
            C223 = smp.Matrix([phi2, theta_eul2, psi3])

            C311 = smp.Matrix([phi3, theta_eul1, psi1])
            C312 = smp.Matrix([phi3, theta_eul1, psi2])
            C313 = smp.Matrix([phi3, theta_eul1, psi3])
            C321 = smp.Matrix([phi3, theta_eul2, psi1])
            C322 = smp.Matrix([phi3, theta_eul2, psi2])
            C323 = smp.Matrix([phi3, theta_eul2, psi3])

            #Compute orientation jacobian
            Jacob_orien111 = C111.jacobian(self._theta)
            Jacob_orien112 = C112.jacobian(self._theta)
            Jacob_orien113 = C113.jacobian(self._theta)
            Jacob_orien121 = C121.jacobian(self._theta)
            Jacob_orien122 = C122.jacobian(self._theta)
            Jacob_orien123 = C123.jacobian(self._theta)

            Jacob_orien211 = C211.jacobian(self._theta)
            Jacob_orien212 = C212.jacobian(self._theta)
            Jacob_orien213 = C213.jacobian(self._theta)
            Jacob_orien221 = C221.jacobian(self._theta)
            Jacob_orien222 = C222.jacobian(self._theta)
            Jacob_orien223 = C223.jacobian(self._theta)

            Jacob_orien311 = C311.jacobian(self._theta)
            Jacob_orien312 = C312.jacobian(self._theta)
            Jacob_orien313 = C313.jacobian(self._theta)
            Jacob_orien321 = C321.jacobian(self._theta)
            Jacob_orien322 = C322.jacobian(self._theta)
            Jacob_orien323 = C323.jacobian(self._theta)

            #Jacobian position
            P = smp.Matrix([T[id_joint][0,3], T[id_joint][1,3], T[id_joint][2,3]]) 
            Jacob_pos = P.jacobian(self._theta)
            
            #final matrices Jacobian
            J111 = smp.Matrix([Jacob_pos, Jacob_orien111])
            J112 = smp.Matrix([Jacob_pos, Jacob_orien112])
            J113 = smp.Matrix([Jacob_pos, Jacob_orien113])
            J121 = smp.Matrix([Jacob_pos, Jacob_orien121])
            J122 = smp.Matrix([Jacob_pos, Jacob_orien122])
            J123 = smp.Matrix([Jacob_pos, Jacob_orien123])

            J211 = smp.Matrix([Jacob_pos, Jacob_orien211])
            J212 = smp.Matrix([Jacob_pos, Jacob_orien212])
            J213 = smp.Matrix([Jacob_pos, Jacob_orien213])
            J221 = smp.Matrix([Jacob_pos, Jacob_orien221])
            J222 = smp.Matrix([Jacob_pos, Jacob_orien222])
            J223 = smp.Matrix([Jacob_pos, Jacob_orien223])

            J311 = smp.Matrix([Jacob_pos, Jacob_orien311])
            J312 = smp.Matrix([Jacob_pos, Jacob_orien312])
            J313 = smp.Matrix([Jacob_pos, Jacob_orien313])
            J321 = smp.Matrix([Jacob_pos, Jacob_orien321])
            J322 = smp.Matrix([Jacob_pos, Jacob_orien322])
            J323 = smp.Matrix([Jacob_pos, Jacob_orien323])
                
            list1 = [J111, J112, J113, J121, J122, J123, J211, J212, J213, J221, J222, J223, J311, J312, J313, J321, J322, J323] 
            final_list.append(list1)
        
        return final_list


    def evaluateJacobian(self, actual_angle, index_joint):
        """
        Select correct jacobian matrix and evaluate it for current joints values \n
        Args:  
            actual_angle (np.array): actual values of angles used to select correct jacobian and computed its inverse
            index_joint: number of joint for which jacobian is calculated
        Return: Jacobian matrix in numpy form for current angles values
        """
        #Select correct Homogeneous and analytical matrices for current joints
        num_matr = self._numpyHM[index_joint]
        analytic_jac = self._numpyAAJ[index_joint]

        #hm = num_matr(actual_angle[0], actual_angle[1], actual_angle[2], actual_angle[3], actual_angle[4], actual_angle[5]) 
        hm = self.evaluateMatrix(num_matr, actual_angle)

        #extract values used to select correct matrix
        r13 = hm[0,2]
        r23 = hm[1,2]
        r33 = hm[2,2]
        r31 = hm[2,0]
        r32 = hm[2,1]
        
        #check value for angle phi
        if(r13 > 0):
            #check value for angle theta
            if(r33 > 0):
                #check value for angle psi
                if(r31 > 0):
                    J = analytic_jac[0]
                else:
                    if(r32 > 0):
                        J = analytic_jac[1]
                    else:
                        J = analytic_jac[2]
            else:
                #check value for angle psi
                if(r31 > 0):
                    J = analytic_jac[3]
                else:
                    if(r32 > 0):
                        J = analytic_jac[4]
                    else:
                        J = analytic_jac[5]
        #check value for angle phi: r13 negative value
        else:
            #check value for angle phi: r23 positive
            if(r23 > 0):
                #check value for angle theta
                if(r33 > 0):
                    #check value for angle psi
                    if(r31 > 0):
                        J = analytic_jac[6]
                    else:
                        if(r32 > 0):
                            J = analytic_jac[7]
                        else:
                            J = analytic_jac[8] 
                else:
                    #check value for angle psi
                    if(r31 > 0):
                        J = analytic_jac[9]
                    else:
                        if(r32 > 0):
                            J = analytic_jac[10]
                        else:
                            J = analytic_jac[11]
            #check value for angle phi: r23 negative
            else:
                #check value for angle theta
                if(r33 > 0):
                    #check value for angle psi
                    if(r31 > 0):
                        J = analytic_jac[12]
                    else:
                        if(r32 > 0):
                            J = analytic_jac[13]
                        else:
                            J = analytic_jac[14] 
                else:
                    #check value for angle psi
                    if(r31 > 0):
                        J = analytic_jac[15]
                    else:
                        if(r32 > 0):
                            J = analytic_jac[16]
                        else:
                            J = analytic_jac[17]

        #Compute jacobian matrix        
        Jacobian = self.evaluateMatrix(J, actual_angle)
        return Jacobian    


    def computeInverseJacobian(self, jacobian):
        """
        Compute inverse Jacobian matrix (Damped Least Squares). \n
        Args: 
            jacobian: Jacobian matrix   
        Return: inverse jacobian matrix 
        """
        #SVD: compute max and min singular value
        u,s,vh = np.linalg.svd(jacobian)
        maxS = np.max(s)
        minS = np.min(s)
        condition_number = maxS/minS
        w = 1./condition_number
        cn_t = 50 #40 #33 #50
        w_t = 1./cn_t
        
        #damped least squares params
        k0 = 1 
        k = 0
        if(w < w_t):
            k = k0 * pow((1-w/w_t), 2)
        #Transpose
        JT = np.transpose(jacobian)
        I = np.identity(6)
        ft = np.dot(jacobian, JT)
        st = pow(k,2) * I
        rt = np.linalg.inv(ft + st)
        Jacob_inv = np.dot(JT, rt)

        return Jacob_inv


    def transformSyminNum(self, matrix):
        """
        Function to transform sympy matrix into numpy matrix to evaluate it faster. \n
        Args:
            matrix: matrix in symbolic form (from sympy)
        Return: numpy matrix
        """
        numpy_form = smp.lambdify(self._theta, matrix, 'numpy')
        return numpy_form


    def get_pose(self, actual_angle, index_joint):
        """
        Get pose matrix. \n
        Args: 
            actual_angle (np.array): actual values of angles (vector q)
            index_joint (int): index of joint
        Return: Pose matrix
        """
         
        hom_matrix = self._numpyHM[index_joint]
        pose_matrix = self.evaluateMatrix(hom_matrix, actual_angle)
        return pose_matrix


    def evaluateMatrix(self, numpy_matrix, actual_angle):
        """
        Evaluate numpy matrix for current angles \n
        Args:
            numpy_matrix: numpy matrix
            actual_angle: values of angles
        Return: numpy matrix
        """
        matr = numpy_matrix(actual_angle[0], actual_angle[1], actual_angle[2], actual_angle[3], actual_angle[4], actual_angle[5])
        return matr


    def getPosition(self, actual_angle, index_joint):
        """
        Get position (x,y,z)
        Args: 
            actual_angle (np.array): actual values of angles (vector q) \n
            index_joint (int): index of joint
        Return: position (x,y,z)
        """
        matrix = self.get_pose(actual_angle, index_joint)
        position = matrix[0:3,3]
        
        return position


