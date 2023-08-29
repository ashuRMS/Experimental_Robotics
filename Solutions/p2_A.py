#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow,sqrt,atan2
import transformations as tf

class go_to_goal:
    def __init__(self):
        # Create a ROS Node 
        rospy.init_node("turtlebot3_controller",anonymous=True)

        # Define the Publihser and Subcriber topic
        pub_topic="/cmd_vel"
        sub_topic="/odom"

        self.pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, Odometry, self.update_pose)

        # Create Odom and Rate Object
        self.odom=Odometry()
        self.rate=rospy.Rate(50)

        # Intialize the Quaternion
        self.quaternion=[0,0,0,1]

    def yaw_angles_from_quaternion(self,quaternion):
        roll,pitch,yaw=list(tf.euler_from_quaternion(quaternion))
        return roll

    def update_pose(self,data):

        self.odom=data
        self.odom.pose.pose.position.x=data.pose.pose.position.x
        self.odom.pose.pose.position.y=data.pose.pose.position.y
        self.odom.pose.pose.orientation.x=data.pose.pose.orientation.x
        self.odom.pose.pose.orientation.y=data.pose.pose.orientation.y
        self.odom.pose.pose.orientation.z=data.pose.pose.orientation.z
        self.odom.pose.pose.orientation.w=data.pose.pose.orientation.w

        self.quaternion=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        

    def euclidean_distance(self,goal_pose):
        return sqrt(pow((goal_pose.pose.pose.position.x- self.odom.pose.pose.position.x),2) + pow((goal_pose.pose.pose.position.y - self.odom.pose.pose.position.y),2))
    
    def linear_vel(self,goal_pose,kp):
        vel=kp*(self.euclidean_distance(goal_pose))
        return max(0.16, min(0.5, vel))
    
    def heading_dirn(self, goal_pose):
        return atan2((goal_pose.pose.pose.position.y - self.odom.pose.pose.position.y), 
                (goal_pose.pose.pose.position.x - self.odom.pose.pose.position.x))

    def heading_error(self,goal_pose):
        q1 = tf.transformations.quaternion_from_euler(0, 0,self.heading_dirn(goal_pose))
        q2 = tf.transformations.quaternion_from_euler(0, 0,self.yaw_angles_from_quaternion(self.quaternion))
        
        # Calculate quaternion difference
        q_diff = tf.transformations.quaternion_multiply(q1, tf.transformations.quaternion_inverse(q2))
        
        # Convert quaternion difference back to angles
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(q_diff)
        angle_diff = yaw
        return angle_diff
    
    def angular_vel(self,goal_pose,kw):
        ang_vel=kw*(self.heading_dirn(goal_pose) - self.yaw_angles_from_quaternion(self.quaternion))  
        return max(0.16, min(0.5, ang_vel))
    
    def going_to_goal(self,x,y):
        goal_pose=Odometry()
    
        goal_pose.pose.pose.position.x= x #1.42
        goal_pose.pose.pose.position.y= y 

        vel_msg=Twist()
        print("Heading error: ",self.heading_error(goal_pose),"quat",self.quaternion,"yaw",self.yaw_angles_from_quaternion(self.quaternion) )

        rospy.sleep(1.0)
        while abs(self.heading_error(goal_pose))>=0.045:
            vel_msg.angular.x=0
            vel_msg.angular.y=0
            vel_msg.angular.z=self.angular_vel(goal_pose,0.25)
            print("Heading error1: ",self.heading_error(goal_pose))
            print("angular vel",vel_msg.angular.z)
        
            self.pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot
        vel_msg.linear.x=0
        vel_msg.angular.z=0
        self.pub.publish(vel_msg)
        rospy.sleep(1.0)

        while self.euclidean_distance(goal_pose) >=0.15:
            print("error",self.euclidean_distance(goal_pose))
            vel_msg.linear.x = self.linear_vel(goal_pose,0.35)
            print("Linear vel",vel_msg.linear.x)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            self.pub.publish(vel_msg)
            error=self.euclidean_distance(goal_pose)
            self.rate.sleep()

        # Stop the robot
        vel_msg.linear.x=0
        vel_msg.angular.z=0
        self.pub.publish(vel_msg)
        rospy.sleep(1.0)

if __name__=="__main__":
    try:
        turtle=go_to_goal()
        rospy.sleep(1.0)
        x=float(input("Enter the x- coordinate: "))
        y=float(input("Enter the y- coordinate: "))
        turtle.going_to_goal(x,y)
        print("Reached goal.", flush=True)
    except rospy.ROSInterruptException:
        pass       
