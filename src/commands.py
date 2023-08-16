#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32MultiArray

rospy.init_node('commands')

joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
target_pub =  rospy.Publisher('/target_goal', Float32MultiArray, queue_size=10)

angles = JointState()
target = Float32MultiArray()


angles.position = [3, 0.2, 0.5, 0.8]  #joints 1,2,3,4
target.data = [-0.226, 0.034, 0.036]  #x,y,z
while not rospy.is_shutdown():
    joint_states_pub.publish(angles)
    target_pub.publish(target)