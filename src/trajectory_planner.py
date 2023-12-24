#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
import numpy as np

def grip():
    gripper_pos_pub.publish(Float64(-0.075))
    gripper_grip_pub.publish(Bool(True))

def release():
    gripper_pos_pub.publish(Float64(0.1))
    gripper_grip_pub.publish(Bool(False))

def third_order_trajectory_planning(tf, thetai, thetaf):
    # Calculate third order polynomial coefficients
    a = np.array([[1,   0,      0,              0],
                  [1,   tf,     tf**2,      tf**3],
                  [0,   1,      0,              0],
                  [0,   1,      2*tf,   3*(tf**2)]])
    b = np.array([[thetai],
                  [thetaf],
                  [0], 
                  [0]])

    poly_coeffs = np.linalg.solve(a, b).reshape(-1).tolist()
    c0, c1, c2, c3 = poly_coeffs[0], poly_coeffs[1], poly_coeffs[2], poly_coeffs[3]
    return c0, c1, c2, c3

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("trajectory_planner_node")

    # Gripper publishers
    gripper_pos_pub = rospy.Publisher("/gripper_position/command", Float64, queue_size= 10, latch=True)
    gripper_grip_pub = rospy.Publisher("/gripper_attach_cmd", Bool, queue_size= 10, latch=True)

    # Create the publishers that'll move the robot's joints
    joint1_pub = rospy.Publisher("/joint1_position/command", Float64, queue_size=10)
    joint2_pub = rospy.Publisher("/joint2_position/command", Float64, queue_size=10)
    joint3_pub = rospy.Publisher("/joint3_position/command", Float64, queue_size=10)
    joint4_pub = rospy.Publisher("/joint4_position/command", Float64, queue_size=10)
    

    # Grip before moving
    release()
    rospy.sleep(2.0)

    c10, c11, c12, c13 = third_order_trajectory_planning(tf=5, thetai=0, thetaf = 0)
    c20, c21, c22, c23 = third_order_trajectory_planning(tf=5, thetai=0, thetaf = 0.228)
    c30, c31, c32, c33 = third_order_trajectory_planning(tf=5, thetai=0, thetaf = -0.128)
    c40, c41, c42, c43 = third_order_trajectory_planning(tf=5, thetai=0, thetaf = -(0.228-0.128))
 
    # Specify dt parameter and calculate discrete time vector
    dt = 0.01   # in seconds
    time_vector = np.linspace(start=0, stop=5, num=int((5-0)/dt))
    joint1_pub.publish(Float64(0))
    joint2_pub.publish(Float64(-1))
    joint3_pub.publish(Float64(-0.3))
    joint4_pub.publish(Float64(0))
    rospy.sleep(2.0)


    for t in time_vector:
        joint1_angle = c10 + c11 * t + c12 * (t**2) + c13 * (t**3)
        joint2_angle = c20 + c21 * t + c22 * (t**2) + c23 * (t**3)
        joint3_angle = c30 + c31 * t + c32 * (t**2) + c33 * (t**3)
        joint4_angle = c40 + c41 * t + c42 * (t**2) + c43 * (t**3)
        print("Joint2 Angle: {}:".format(joint2_angle))
        print("Joint3 Angle: {}:".format(joint3_angle))
        print("Joint4 Angle: {}:".format(joint4_angle))
        joint1_pub.publish(Float64(joint1_angle))
        joint2_pub.publish(Float64(joint2_angle))
        joint3_pub.publish(Float64(joint3_angle))
        joint4_pub.publish(Float64(joint4_angle))
        rospy.sleep(dt)


    # Release upon arrival
    grip()
    rospy.sleep(1.0)


    # c20, c21, c22, c23 = third_order_trajectory_planning(tf=5, thetai=0.228, thetaf=-1)
    # c30, c31, c32, c33 = third_order_trajectory_planning(tf=5, thetai=-0.128, thetaf=0.7)
    # c40, c41, c42, c43 = third_order_trajectory_planning(tf=5, thetai=-0.1, thetaf=0.3)

    # for t in time_vector:
    #     joint2_angle = c20 + c21 * t + c22 * (t**2) + c23 * (t**3)
    #     joint4_angle = c40 + c41 * t + c42 * (t**2) + c43 * (t**3)
    #     joint3_angle = c30 + c31 * t + c32 * (t**2) + c33 * (t**3)
    #     print("Joint2 Angle: {}:".format(joint2_angle))
    #     print("Joint3 Angle: {}:".format(joint3_angle))
    #     print("Joint4 Angle: {}:".format(joint4_angle))
    #     joint2_pub.publish(Float64(joint2_angle))
    #     joint3_pub.publish(Float64(joint3_angle))
    #     joint4_pub.publish(Float64(joint4_angle))
    #     rospy.sleep(dt)
    


    c10, c11, c12, c13 = third_order_trajectory_planning(tf=5, thetai=0, thetaf=-np.pi/2)
    c20, c21, c22, c23 = third_order_trajectory_planning(tf=5, thetai=0.228, thetaf=-0.3)
    c30, c31, c32, c33 = third_order_trajectory_planning(tf=5, thetai=-0.128, thetaf=0)
    c40, c41, c42, c43 = third_order_trajectory_planning(tf=5, thetai=-0.1, thetaf=0.3)

    for t in time_vector:
        joint1_angle = c10 + c11 * t + c12 * (t**2) + c13 * (t**3)
        joint2_angle = c20 + c21 * t + c22 * (t**2) + c23 * (t**3)
        joint3_angle = c30 + c31 * t + c32 * (t**2) + c33 * (t**3)
        joint4_angle = c40 + c41 * t + c42 * (t**2) + c43 * (t**3)
        print("Joint2 Angle: {}:".format(joint2_angle))
        print("Joint3 Angle: {}:".format(joint3_angle))
        print("Joint4 Angle: {}:".format(joint4_angle))
        joint1_pub.publish(Float64(joint1_angle))
        joint2_pub.publish(Float64(joint2_angle))
        joint3_pub.publish(Float64(joint3_angle))
        joint4_pub.publish(Float64(joint4_angle))
        rospy.sleep(dt)
    
    release()
    # Sleep sometime for stabilization before termination
    rospy.sleep(2.0)

