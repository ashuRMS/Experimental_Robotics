#!/usr/bin/env python3
import rospy
import numpy as np
import sympy as sp
from sympy import Matrix, N
import math
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates

class JointPositionControlNode:
    def __init__(self):
        rospy.init_node("joint_position_control_node")
        self.service_name = 'goal_joint_space_path'  # Replace with the actual service name
        self.current_position = []  # Initialize initial_joint_angles as None
        self.current_angles=[]
        # Create a subscriber to listen to the topic current initial joint angles

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.current_position_callback)
        rospy.Subscriber("/joint_states",JointState,self.joint_state_callback)

    def joint_state_callback(self,data):
        self.current_angles=list(data.position[2:])
        # print("joint angles",self.current_angles)
        


    def current_position_callback(self, data):
        # Callback function to store initial joint angles
        gripper_link_sub_index = data.name.index('open_manipulator::gripper_link_sub')
        x_position = data.pose[gripper_link_sub_index].position.x + 0.045
        y_position = data.pose[gripper_link_sub_index].position.y
        z_position = data.pose[gripper_link_sub_index].position.z
        self.current_position=[x_position,y_position,z_position]
        # print("current_position",self.current_position)


    def set_position(self, j1,j2,j3,j4,time_step):

        try:
            #rospy.wait_for_service(self.service_name, timeout=10.0)  # Adjust the timeout as needed
            set_position = rospy.ServiceProxy(self.service_name, SetJointPosition)
            arg = SetJointPositionRequest()
            arg.planning_group = "your_planning_group"
            arg.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
            arg.joint_position.position = [j1, j2, j3, j4]
            arg.path_time = time_step # Replace with your desired path time
            # THis made me struggle a lot
            resp1 = set_position(arg)
            rospy.loginfo('Service done!')
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


def forward_kinematics_inv_Jacobian(q):         #return ee_pos, np.linalg.pinv(Jv)
    # Function for forward kinematics and inverse Jacobian calculation
    
    epsilon = sp.Matrix([1, 1, 1, 1])
    n = 4

    # Define symbolic variables
    theta = sp.symbols('theta[0:%d]' % n)
    alpha = np.array([0, math.pi/2, 0, 0])
    a = np.array([0.012, 0, 0.130, 0.124])
    d = np.array([0.077, 0, 0, 0])

    # Initialize the transformation matrices
    Ttemp = sp.eye(4)
    HTM = [sp.eye(4) for _ in range(n)]

    for i in range(n):
        # Calculate the transformation matrix elements using symbolic variables
        t11 = sp.cos(theta[i])
        t12 = -sp.sin(theta[i])
        t13 = 0
        t14 = a[i]

        t21 = sp.sin(theta[i]) * sp.cos(alpha[i])
        t22 = sp.cos(theta[i]) * sp.cos(alpha[i])
        t23 = -sp.sin(alpha[i])
        t24 = -d[i] * sp.sin(alpha[i])

        t31 = sp.sin(theta[i]) * sp.sin(alpha[i])
        t32 = sp.cos(theta[i]) * sp.sin(alpha[i])
        t33 = sp.cos(alpha[i])
        t34 = d[i] * sp.cos(alpha[i])

        Tiim1 = sp.Matrix([[t11, t12, t13, t14],
                           [t21, t22, t23, t24],
                           [t31, t32, t33, t34],
                           [0, 0, 0, 1]])

        Ti0 = Ttemp * Tiim1
        HTM[i] = Ti0
        Ttemp = Ti0

    # Homogeneous Transformation matrix
    Tn0 = HTM[n - 1]
    HTM_list = [HTM[0], HTM[1], HTM[2], HTM[3]]

    # End effector's position
    pE0_h = Tn0 * sp.Matrix([0.126, 0, 0, 1])
    pE0 = sp.Matrix(pE0_h[:3])

    # Initialize the Jacobian matrix
    Jv = sp.Matrix.ones(3, n)

    for i, item in enumerate(HTM_list):
        v1 = epsilon[i] * sp.Matrix(item[:3, 2])
        v2 = pE0 - sp.Matrix(item[:3, 3])
        cross_product = v1.cross(v2)
        Jv[:, i] = cross_product

    # Substitute joint values into the Jacobian matrix
    theta_values = [q[0], q[1] + math.pi/2 - 0.1853, q[2] - math.pi/2 + 0.1853, q[3]]
    Jv_substituted = Jv.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
                              theta[2]: theta_values[2], theta[3]: theta_values[3]})
    Jv_substituted = N(Jv_substituted, 3)
    Jv_substituted = Jv_substituted.applyfunc(lambda x: round(x, 3))
    
    # Substitute joint values into end effector position
    ee_pos = pE0.subs({theta[0]: theta_values[0], theta[1]: theta_values[1],
                      theta[2]: theta_values[2], theta[3]: theta_values[3]})
    Jv = np.array(Jv_substituted, dtype=float)
    ee_pos = np.array(ee_pos, dtype=float).reshape(3)
    
    return np.linalg.pinv(Jv)



if __name__ == "__main__":
    try:
        # print("1")
        joint_position_node = JointPositionControlNode()
        # print("2")
        rospy.sleep(0.5)
        
        angles=joint_position_node.current_angles
 
        x_start, y_start, z_start = joint_position_node.current_position[0],joint_position_node.current_position[1],joint_position_node.current_position[2]
        # print(x_start,y_start,z_start)
        q = np.array(angles, dtype=float)
        print("topic angles: ",q)
        x_end, y_end, z_end = 0.134, 0.0, 0.241
        # exit()
        # # Number of points you want to generate
        num_points = 120

        # Generate sample points
        x_points = np.linspace(x_start, x_end, num_points)
        y_points = np.linspace(y_start, y_end, num_points)
        z_points = np.linspace(z_start, z_end, num_points)

        # Create a list to store the points
        points_list = []

        # Initialize lists to store joint angles and errors over time
        joint_angles_over_time = []
        errors_over_time = []

        # print("2")
        for i in range(num_points):
            point = [x_points[i], y_points[i], z_points[i]]
            points_list.append(point)

        Jinv = forward_kinematics_inv_Jacobian(q)

        for i in range(1, len(points_list)):
            next_pos = np.array(points_list[i])
            error = next_pos - joint_position_node.current_position
            Euclideanerror = np.linalg.norm(np.array(points_list[-1]) - joint_position_node.current_position)

            mse = np.linalg.norm(error)
            
            joint_angles_over_time.append(q.copy())
            errors_over_time.append(Euclideanerror)
            # print("mse")
            while mse > 0.01:
                dq = np.dot(Jinv, error)
                q += 0.08*dq
                joint_position_node.set_position(q[0],-q[1],-q[2],-q[3],0.01)
                # rospy.sleep(0.05)
                print("-------------------------------")
                
                # print("next_angles",q)
                # print("\ntopic_angles",angles)
                # print("\nnext_pos: ",next_pos)
                print("\ntopic_pose:",joint_position_node.current_position)
                # print("-------------------------------")
                # exit()
                
                # print("4")
                
                print("Joint angles",q)
                Jinv = forward_kinematics_inv_Jacobian(q)
                error = next_pos - joint_position_node.current_position
                mse = np.linalg.norm(error)
                print(mse)
        # After all the processing, include rospy.spin() to keep the node alive
        print("Done")
        exit()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted.")
