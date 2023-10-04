#!/usr/bin/env python3
import rospy
from beginner_tutorials.msg import End_effector_pose,Output_joint_positions
import numpy as np
np.set_printoptions(precision=4)
# Intialize the inverse_kinematics Node
rospy.init_node('inverse_kinematics_X',anonymous=True)
pE0=None

# Create a callback function
def topic_callback(data):
    global pE0
    pE0=data
    target_pos=[data.x,data.y,data.z,data.phi]
    omx_inverse_kinematics(target_pos)

# Create Publisher and Subscriber Node
rospy.Subscriber("End_effector_pose",End_effector_pose,topic_callback)
joint_angles_pub = rospy.Publisher("Output_joint_position", Output_joint_positions, queue_size=10)


# Calculate Inverse Kinematics for 3R robot
def P3R_inverse_kinematics(a1, a2, a3, x3, y3, theta):

    x2 = x3 - a3*np.cos(theta)
    y2 = y3 - a3*np.sin(theta)

    cos_th2 = (x2**2+y2**2-a1**2-a2**2)/(2*a1*a2)

    sin_th2 = [-np.sqrt(1-cos_th2**2), np.sqrt(1-cos_th2**2)]

    th2 = [np.arctan2(sin_th2[0], cos_th2),np.arctan2(sin_th2[1], cos_th2)]

    sin_th1 = [(y2*(a1+a2*np.cos(th2[0]))-a2*np.sin(th2[0])*x2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[0])),
               (y2*(a1+a2*np.cos(th2[1]))-a2*np.sin(th2[1])*x2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[1]))]
    
    cos_th1 = [(x2*(a1+a2*np.cos(th2[0]))+a2*np.sin(th2[0])*y2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[0])),
               (x2*(a1+a2*np.cos(th2[1]))+a2*np.sin(th2[1])*y2)/(a1**2+a2**2+2*a1*a2*np.cos(th2[1]))]
    
    th1 = [np.arctan2(sin_th1[0], cos_th1[0]),np.arctan2(sin_th1[1], cos_th1[1])]

    th3 = [theta-th1[0]-th2[0], theta-th1[1]-th2[1]]

    return th1, th2, th3

# Inverse Kinematics for Open Manipulator X
def omx_inverse_kinematics(target_pos):
    
    # End effector pose
    x = target_pos[0]- 0.012
    y = target_pos[1]
    z = target_pos[2]
    phi = -target_pos[3]
    
    # D-H parameters
    d1 =0.077
    a1 = np.sqrt(0.024**2+0.128**2)
    alpha_2 = np.arctan(0.024/0.128)
    a2 = 0.124
    a3 = 0.126

    # Transformation from x-y-z plane to robots plane
    x_new = np.sqrt(x**2+y**2)
    y_new = z-d1

    
    # First Joint Angle
    theta_1 = [np.arctan2(y, x), np.arctan2(-y, -x)]

    # Array to store all other joint angles
    thetas = []
    for i in range(1):
        th2, th3, th4 = P3R_inverse_kinematics(a1=a1,a2=a2,a3=a3,x3=x_new,y3=y_new,theta=phi)
        
        # Different set of solutions for inverse kinematics
        theta_2 = [np.pi/2-th2[0]-alpha_2, np.pi/2-th2[1]-alpha_2]
        theta_3 = [-np.pi/2-th3[0]+alpha_2,-np.pi/2-th3[1]+alpha_2]
        theta_4 = [-th4[0], -th4[1]]

        thetas.append([theta_1[i], theta_2[0], theta_3[0], theta_4[0]])
        thetas.append([theta_1[i], theta_2[1], theta_3[1], theta_4[1]])

    # Create a publish message object
    jangles=Output_joint_positions()

    # Publsih the first set of joint angles
    jangles.joint1_position = thetas[0][0]
    jangles.joint2_position = thetas[0][1]
    jangles.joint3_position = thetas[0][2]
    jangles.joint4_position = thetas[0][3]

    joint_angles_pub.publish(jangles)
    print("******************\n")
    print("Calculated Joint angles: 1st Set :: {:.3f}, {:.3f}, {:.3f},{:.3f}\n".format(thetas[0][0], thetas[0][1], thetas[0][2],thetas[0][3]))
    print("Calculated Joint angles: 2nd Set :: {:.3f}, {:.3f}, {:.3f},{:.3f}\n".format(thetas[1][0], thetas[1][1], thetas[1][2],thetas[1][3]))
    print("******************\n")


if __name__=="__main__":
    try:
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

