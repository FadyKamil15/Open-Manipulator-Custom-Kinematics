#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float64

rospy.init_node("ikine_node")


#defining publishers

joint1_pub = rospy.Publisher("/joint1_position/command", Float64, queue_size=10)
joint2_pub = rospy.Publisher("/joint2_position/command", Float64, queue_size=10)
joint3_pub = rospy.Publisher("/joint3_position/command", Float64, queue_size=10)



joint1_msg = Float64()
joint2_msg = Float64()
joint3_msg = Float64()



#to calculate the right angle taking into account its quadrant
def modified_arctan(x, y):
    theta  = np.arctan(y/x)
    if (x < 0 and y < 0):
        theta += np.pi
    elif (x > 0 and y < 0):
        theta *= -1
    elif (x < 0 and y > 0):
        theta += np.pi
        


    return theta

def inverse_kinematics(x, y, z):
    x -= 0.012
    joint1_angle = modified_arctan(x, y)
    r = np.sqrt(x**2 + y**2)    #done
    m = np.sqrt((z - 0.077) ** 2 + r **2)   #done
    
    alpha = np.arctan((z - 0.077) / r)
    beta = np.arccos( (0.13**2 + m**2 - 0.25**2) / (2 * 0.130 * m))

    joint2_angle = alpha + beta
    
    joint3_angle = np.pi - np.arccos( (0.13**2 - m**2 + 0.25**2) / (2 * 0.130 * 0.25))
    return joint1_angle, joint2_angle, joint3_angle



def clbk_fn (recieved_msg):
    target_x = recieved_msg.data[0]
    target_y = recieved_msg.data[1]
    target_z = recieved_msg.data[2]
    joint1_msg, joint2_msg, joint3_msg= inverse_kinematics(target_x, target_y, target_z)

    joint1_pub.publish(joint1_msg)
    joint2_pub.publish((joint2_msg - np.arctan( 0.128/ 0.024)) * -1)
    joint3_pub.publish(joint3_msg - np.arctan( 0.128/ 0.024))

    return

rospy.Subscriber("/target_goal",Float32MultiArray , clbk_fn)

while not rospy.is_shutdown():
    pass