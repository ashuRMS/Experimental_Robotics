#!/usr/bin/env python3
import rospy
import numpy as np
from beginner_tutorials.msg import Joint_position,End_effector_position
import math                                                                             # SEE ANGLES ARE IN RADIANS OR DEGREES

# Initialize Nodes
rospy.init_node('forward_kinematics',anonymous=True)

# global variable to store joint positions
joint_array=None

# define callback function
def topic_callback(data):
    global joint_array
    joint_array=data
    forward_kinematics(joint_array)
    # print(joint_array,end="/n")

# Define Publisher and Subscriber
rospy.Subscriber("Input_Joint_position", Joint_position, topic_callback)
end_effector_pub=rospy.Publisher("End_effector_position", End_effector_position, queue_size=10)

# Function to calculate forward kinematics
def forward_kinematics(joint_array):
    # Load joint angles
    theta1,theta2,theta3,theta4=joint_array.joint1_position,joint_array.joint2_position,joint_array.joint3_position,joint_array.joint4_position
    # print(theta1,theta2,theta3,theta4)
    
    # A constant from D-H parameter table
    theta0=0.1853

    # Number of Joints
    n=4
    
    # D-H parameters
    alp=np.array([0,math.pi/2, 0,0])
    a=np.array([0.012,0,0.130,0.124]) # 0.012 because of gazebo
    d=np.array([0.077,0,0,0])
    # theta=np.array([math.radians(theta1),math.radians(theta2) +math.pi/2 - theta0 ,math.radians(theta3) - math.pi/2 + 0.1853 ,math.radians(theta4)])
    theta=np.array([math.radians(theta1),-math.radians(theta2) +math.pi/2 - theta0 ,-math.radians(theta3) - math.pi/2 + 0.1853 ,-math.radians(theta4)]) # -> for matching with gazebo
    # gazebo has different sign convention for angles
   
    # End effector's position
    p4E_4=np.array([0.126,0,0])

    # Initialization Tranformation matrix
    Ttemp=np.eye(4)
    HTM=np.zeros((4,4,n))
    for i in range(n):

        t11=np.cos(theta[i])
        t12=-np.sin(theta[i])
        t13=0
        t14=a[i]

        t21=np.sin(theta[i])*np.cos(alp[i])
        t22=np.cos(theta[i])*np.cos(alp[i])
        t23=-np.sin(alp[i])
        t24=-d[i]*np.sin(alp[i])

        t31=np.sin(theta[i])*np.sin(alp[i])
        t32=np.cos(theta[i])*np.sin(alp[i])
        t33=np.cos(alp[i])
        t34=d[i]*np.cos(alp[i])

        Tiim1=np.array([[t11, t12, t13, t14],
                        [t21,t22,t23,t24],
                        [t31,t32,t33,t34],
                        [0,0,0,1]])
        
        Ti0=np.dot(Ttemp,Tiim1)
        HTM[:,:,i]=Ti0
        Ttemp=Ti0

    # Homogeneous Transformation matrix
    Tn0=HTM[:,:,n-1]

    # End effector's position
    pE0_h=np.dot(Tn0,np.append(p4E_4,1))
    pE0=pE0_h[:3]
    
    # For publishing on custom topic
    pose=End_effector_position()
    pose.x,pose.y,pose.z=pE0[0],pE0[1],pE0[2]
    end_effector_pub.publish(pose)

if __name__=="__main__":
    try:
        # I can listen to everymessage
        rospy.spin()   
    except rospy.ROSInterruptException:
        pass


