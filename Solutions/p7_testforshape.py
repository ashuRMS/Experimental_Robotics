#!/usr/bin/env python3
import rospy
import numpy as np
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

class JointPositionControlNode:
    def __init__(self):
        rospy.init_node("joint_position_control_node")
        self.service_name = 'goal_joint_space_path'  # Replace with the actual service name
        self.initial_angles = None  # Initialize initial_joint_angles as None
        # Create a subscriber to listen to the topic publishing initial joint angles

        rospy.Subscriber("/initial_joint_angles", JointState, self.joint_state_callback)


    def joint_state_callback(self, data):
        # Callback function to store initial joint angles
        self.initial_angles = data.position[:4]
        # print("intial_angles",self.initial_angles)

    def set_position(self, j1,j2,j3,j4,time_step):

        try:
            #rospy.wait_for_service(self.service_name, timeout=10.0)  # Adjust the timeout as needed
            set_position = rospy.ServiceProxy(self.service_name, SetJointPosition)
            arg = SetJointPositionRequest()
            arg.planning_group = "your_planning_group"
            arg.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
            arg.joint_position.position = [j1, j2, j3, j4]
            arg.path_time = time_step/6 # Replace with your desired path time
            # THis made me struggle a lot
            resp1 = set_position(arg)
            rospy.loginfo('Service done!')
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    # Function to generate cycloidal trajectories for all joint angles
    def generate_cycloidal_trajectories(self,initial_angles, final_angles, total_time, num_points):
        times = np.linspace(0, total_time, num_points)
        c2 = np.pi / total_time
        time_step=total_time/len(times)

        joint_trajectories = []
        for initial_angle, final_angle in zip(initial_angles, final_angles):
            # Calculate the joint angles based on cycloidal trajectory for each joint
            joint_angles = initial_angle + (final_angle - initial_angle) * (0.5 - 0.5 * np.cos(c2 * times))
            joint_trajectories.append(joint_angles)

        plt.figure()
        for i in range(4):  # Loop through each joint
            plt.plot(times, joint_trajectories[i], label=f'Joint {i+1}')

        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angles (radians)')
        plt.title('Cycloidal Joint Trajectories')
        plt.legend()
        plt.grid(True)
        plt.show()


        return time_step,times, joint_trajectories
    

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

    return thetas[0]

if __name__ == "__main__":
    try:
        joint_position_node = JointPositionControlNode()

        # Initialize initial angles as zeros
        initial_angles = [0.0, 0.0, 0.0, 0.0]

        while True:
            # Prompt the user to provide the end-effector position (x, y, z)
            x, y, z, phi = [float(coord) for coord in input("Enter final end-effector position (x, y, z) as comma-separated values: ").split(",")]
            total_time = float(input("Enter the total time: "))
            num_points = 100
        

            # Get the joint angles 
            final_angles=omx_inverse_kinematics([x,y,z,phi])
            # print(final_angles)
            # Generate and set the joint angles
            time_step,times, joint_trajectories = joint_position_node.generate_cycloidal_trajectories(initial_angles, final_angles, total_time,num_points)

            for i in range(len(joint_trajectories[0][:])):
       
                print(joint_trajectories[0][i],joint_trajectories[1][i],joint_trajectories[2][i],joint_trajectories[3][i])
                joint_position_node.set_position(joint_trajectories[0][i],joint_trajectories[1][i],joint_trajectories[2][i],joint_trajectories[3][i],time_step)
    

            # Update the initial angles for the next iteration
            initial_angles = final_angles

    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted.")
