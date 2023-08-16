#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


 #initializing a node, defining a subscriber
rospy.init_node('fkine_node')
fkine_pub = rospy.Publisher("/robot_pose", Float32MultiArray, queue_size=10)

def dh_matrix_creator(theta, d, alpha, a):
    alpha = np.radians(alpha)
    dh_matrix = np.array([[np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
                         [np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha)*np.cos(theta), a*np.sin(theta)],
                         [0,             np.sin(alpha),                np.cos(alpha),               d],
                         [0,             0,                            0,                           1]])

    return dh_matrix

def dh_to_euler_creator(dh_matrix):
    m = dh_matrix
    x = m[0][3] + 0.012
    y = m[1][3]
    z = m[2][3]
    roll = np.arctan(m[2][1] / m[2][2])
    pitch = np.arctan(m[2][0] * -1 / np.sqrt(m[2][1] ** 2 + m[2][2] ** 2))
    yaw = np.arctan(m[1][0] / m[0][0])
    
    return x,y,z,roll,pitch,yaw

def clbk_fn(received_msg):
    # this will be called each time the topic
    # receives a message
    angles = received_msg.position
    #assigning joint angles to calculate DH transformation matrices
    theta1 = angles[0]
    theta2 = angles[1]
    theta3 = angles[2]
    theta4 = angles[3]


    #defining publishing message
    sixdof = Float32MultiArray()


    #accounting for the positive rotation direction and initial position
    theta2 -= np.arctan(0.128/0.024) 
    theta3 += np.arctan(0.128/0.024) 
    theta4 *= -1


    trans_0_1 = dh_matrix_creator(theta1,0.077,-90,0)
    trans_1_2 = dh_matrix_creator(theta2,0,0,0.130)
    trans_2_3 = dh_matrix_creator(theta3,0,180,0.124)
    trans_3_E = dh_matrix_creator(theta4,0,0,0.126)
    trans_0_E = trans_0_1 @ trans_1_2 @ trans_2_3 @ trans_3_E  

    sixdof.data = dh_to_euler_creator(trans_0_E)
    fkine_pub.publish(sixdof)
    return 




#defining a subscriber
rospy.Subscriber("/joint_states", JointState, clbk_fn)


#publishing
while not rospy.is_shutdown():
    pass