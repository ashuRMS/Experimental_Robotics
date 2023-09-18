import numpy as np
import math
import csv

filepath = '/home/ashutoshsahu/catkin_ws/src/beginner_tutorials/scripts/practical4/true_joint_positions.csv'

def process_csv_file(filename):
    joint_angles_list = []
    true_end_effector_list = []

    with open(filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        # next(csv_reader)  # Skip the header if present

        for row in csv_reader:
            # Extract joint angles (first four values) and true end effector pose (next three values)
            joint_angles = [float(value) for value in row[:4]]
            true_end_effector = [float(value) for value in row[4:7]]

            # Append the values to the respective lists
            joint_angles_list.append(joint_angles)
            true_end_effector_list.append(true_end_effector)

    return joint_angles_list, true_end_effector_list


joint_angles, true_end_effector = process_csv_file(filepath)
# print(true_end_effector)
print("\n")
# print(joint_angles)

def forward_kinematics(joint_angle):
    theta1,theta2,theta3,theta4=joint_angle[0],joint_angle[1],joint_angle[2],joint_angle[3]
    

    # A constant from D-H parameter table
    theta0=0.1853

    # Number of Joints
    n=4
    
    # D-H parameters
    alp=np.array([0,math.pi/2, 0,0])
    a=np.array([0.012,0,0.130,0.124]) # 0.012 because of gazebo
    d=np.array([0.077,0,0,0])
    # theta=np.array([math.radians(theta1),math.radians(theta2) +math.pi/2 - theta0 ,math.radians(theta3) - math.pi/2 + 0.1853 ,math.radians(theta4)])
    theta=np.array([theta1,-theta2 +math.pi/2 - theta0 ,-theta3 - math.pi/2 + 0.1853 ,-theta4]) # -> for matching with gazebo
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
   
    return pE0
   
   
def calculate_end_effector_error(joint_angles, true_end_effector):
    calculated_pos=[]
    # print(calculated_pos)

    for joint in joint_angles:
        calculated_pos.append(list(forward_kinematics(joint)))


    # Initialize a list to store the Euclidean norms
    euclidean_norms = []
    

    # Iterate through the sublists and calculate Euclidean norms
    for sublist_a, sublist_b in zip(calculated_pos, true_end_effector):
        squared_differences = [(x - y)**2 for x, y in zip(sublist_a, sublist_b)]
        euclidean_norm = math.sqrt(sum(squared_differences))
        euclidean_norms.append(euclidean_norm)
    # print(euclidean_norms)
        
    for i in range(0,5):
        # Print the results
        print("****************************************\n")
        print("True End Effector Pose:", true_end_effector[i])
    
        print("Calculated End Effector Pose:", calculated_pos[i])

        print("Error:", euclidean_norms[i])
        print("---------------------------------\n")

calculate_end_effector_error(joint_angles,true_end_effector)