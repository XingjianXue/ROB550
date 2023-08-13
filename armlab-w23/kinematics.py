"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
import scipy
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as R
import math
import copy


def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    trans_matrix_base_to_link = np.identity(4)
    dh_params = copy.deepcopy(dh_params)

    for i in range(link):
        if i == 1:
             dh_params[1][3] -= joint_angles[1] 
        else:
            dh_params[i][3] += joint_angles[i]
    for i in range(link):
        dh_param = dh_params[i]
        A = get_transform_from_dh(dh_param[0]*1e-3, dh_param[1], dh_param[2]*1e-3, dh_param[3])
        trans_matrix_base_to_link = np.matmul(trans_matrix_base_to_link, A)

    return trans_matrix_base_to_link
    

def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    rot_theta = np.array([[np.cos(theta), -np.sin(theta), 0, 0],[np.sin(theta), np.cos(theta), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
    trans_d = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, d],[0, 0, 0, 1]])
    trans_a = np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    rot_alpha = np.array([[1, 0, 0, 0],[0, np.cos(alpha), -np.sin(alpha), 0],[0, np.sin(alpha), np.cos(alpha), 0],[0, 0, 0, 1]])
    A = np.matmul(rot_theta, np.matmul(trans_d, np.matmul(trans_a, rot_alpha)))
    return A


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    r = R.from_dcm(T[:3,:3])
    return r.as_euler('ZYZ', degrees=False)


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    # print(T[1,1])
    # print(T[0,3])
    x = T[0,3]
    y = T[1,3]
    z = T[2,3]
    phi, theta, psi = get_euler_angles_from_T(T)
    return np.array([x,y,z,phi, theta, psi])


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    pass


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    x = copy.deepcopy(pose[0])
    y = copy.deepcopy(pose[1])
    z = copy.deepcopy(pose[2]) 
    psi = copy.deepcopy(pose[3])

    theta0 = -np.arctan2(x,y)

    l1 = 103.09
    l2 = 205
    l3 = 200
    l4 = 174
    x_ = np.sqrt(np.square(x)+np.square(y))-l4*np.cos(psi)
    y_ = z - l4*np.sin(psi)-l1
    if(np.sqrt(np.square(x_)+np.square(y_)) < l2+l3):
        theta2_1 = np.arccos(((np.square(x_)+np.square(y_))-(np.square(l2)+np.square(l3)))/(2*l2*l3))
        theta2_2 = -np.arccos(((np.square(x_)+np.square(y_))-(np.square(l2)+np.square(l3)))/(2*l2*l3))
        theta1_1 = np.arctan2(y_,x_) - np.arctan2(l3*np.sin(theta2_1),l2 + l3*np.cos(theta2_1))
        theta1_2 = np.arctan2(y_,x_) - np.arctan2(l3*np.sin(theta2_2),l2 + l3*np.cos(theta2_2))
        theta3_1 = psi - (theta1_1+theta2_1)
        theta3_2 = psi - (theta1_2+theta2_2)
        angle1 = [theta0-0.05,np.pi/2-theta1_1-0.245,theta2_1+1.325,theta3_1 + 0.15,0]
        angle2 = [theta0-0.05,np.pi/2-theta1_2-0.245,theta2_2+1.325,theta3_2 + 0.15,0]
        if (pose[3] == -np.pi/2):
            angle1[4] = pose[4]
            angle2[4] = pose[4]

        return angle1, angle2
    elif(np.sqrt(np.square(x_)+np.square(y_)) > l2+l3 and pose[3] <= 0):
        pose[3] = pose[3] + np.pi/8
        angle1, angle2 = IK_geometric(pose)
        return angle1,angle2

 


        




        
        


        
    
    





   