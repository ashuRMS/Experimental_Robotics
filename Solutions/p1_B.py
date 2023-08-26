#!/usr/bin/env python3
# RIGHT CODE
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,sqrt
import transformations as tf
import math

class go_to_goal:
    def __init__(self):
        # Intialize the the turtle_controller node
        rospy.init_node('turtle_controller',anonymous=True)   

        # Create Publisher and Subscriber object
        pub_topic="/turtle1/cmd_vel"                          
        sub_topic="/turtle1/pose"                            

        self.pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, Pose, self.update_pose)

        # Create pose object
        self.pose=Pose()                                       
        self.rate =rospy.Rate(50)
        
    def update_pose(self, data):
        # Update the pose from the received data DATA RECEIVED CORRECTLY
        # print("Received pose data:", data)
        self.pose.x = data.x
        self.pose.y = data.y
        self.pose.theta = data.theta  
        # print("Updated self.pose:", self.pose)
        
    
    def euclidean_distance(self,goal_pose):
        # Calculate the euclidean distance between desired and current position
        return sqrt(pow((goal_pose.x - self.pose.x),2) + pow((goal_pose.y - self.pose.y),2))
    
    def linear_vel(self,goal_pose,kp):
        # Calculate linear velocity based on euclidean distance. Kp is gain parameter
        return kp*(self.euclidean_distance(goal_pose))
    
    def heading_dirn(self,goal_pose):
        # Calculate the heading diretion
        delta_y = goal_pose.y - self.pose.y
        delta_x = goal_pose.x - self.pose.x
        return math.atan2(delta_y, delta_x)
    
    def calculate_angle_difference(self,angle2, angle1): 
        # Create quaternion from angle1 and angle2
        q1 = tf.transformations.quaternion_from_euler(0, 0,angle1)
        q2 = tf.transformations.quaternion_from_euler(0, 0,angle2)
        
        # Calculate quaternion difference
        q_diff = tf.transformations.quaternion_multiply(q2, tf.transformations.quaternion_inverse(q1))
        
        # Convert quaternion difference back to angles
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(q_diff)
        angle_diff = yaw
        return angle_diff
    
    def angular_vel(self,goal_pose,kw):
        # Calculate the angular velocities
        return kw*self.calculate_angle_difference(self.heading_dirn(goal_pose),self.pose.theta)

    def going_to_goal(self,x,y):
        goal_pose=Pose()

        goal_pose.x= x #1.42
        goal_pose.y= y  #9
        
        # Tolerance
        error=0.001

        # Create Twist Object
        vel_msg=Twist() 
 
        # Move until the end   
        rospy.sleep(1.0)                                   
        while self.euclidean_distance(goal_pose)>=error:
            vel_msg.linear.x = self.linear_vel(goal_pose,1.5)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x=0
            vel_msg.angular.y=0
            vel_msg.angular.z=self.angular_vel(goal_pose,10)
            self.pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot
        vel_msg.linear.x=0
        vel_msg.angular.z=0
        self.pub.publish(vel_msg)                               # Publishing velocity commands

        rospy.sleep(1.0)
        print(f"Euclidean distance Error: {self.euclidean_distance(goal_pose):.4f}, (X, Y) = ({self.pose.x:.4f}, {self.pose.y:.4f})")

    def orient_myway(self,goal_theta):
        # This function orients the turtle in desired orientation which was not possible with going_to_goal function
        goal_pose=Pose()
        goal_pose.theta=goal_theta
        vel_msg=Twist()
        while goal_pose.theta - self.pose.theta >=0.001:
            vel_msg.angular.z=3*(goal_pose.theta-self.pose.theta)
            self.pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x=0
        vel_msg.angular.z=0
        self.pub.publish(vel_msg)                               # Publishing velocity commands
        print(f"Heading Error: {goal_pose.theta - self.pose.theta:.4f}")

if __name__=="__main__":
    try:

        x=float(input("Enter the x-coordinate: "))
        y=float(input("Enter the y-coordinate: "))
        turtle=go_to_goal()
        print("************************************************************")
        turtle.going_to_goal(x,y)
        print("Reached first goal. \n", flush=True)
        print("************************************************************")
        # turtle.going_to_goal(y,x)
        # print("Reached second goal. \n", flush=True)
        # print("************************************************************")
        # turtle.going_to_goal(5.45,5.45)
        # print("Reached third goal. \n", flush=True)
        # print("************************************************************")
        # turtle.orient_myway(1.57)
        # print("Oriented as per my way \n",flush=True)
    except rospy.ROSInterruptException:
        pass       
        
