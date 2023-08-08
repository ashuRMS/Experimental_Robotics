#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,sqrt,atan2
import roslaunch

class go_to_goal:
    def __init__(self):
        rospy.init_node('turtle_controller',anonymous=True)   # Intialize the the turtle_controller node

        pub_topic="/turtle1/cmd_vel"                          # Publisher topic
        sub_topic="/turtle1/pose"                             # Subscriber topic

        self.pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, Pose, self.update_pose)

        self.pose=Pose()                                       # pose object
        self.rate =rospy.Rate(20)
    
    def update_pose(self,data):
        self.pose=data
        # The data parameter is already being passed in the subscriber object. This is the pecuilarity of ROS framework
        self.pose.x=round(self.pose.x,4)
        self.pose.y=round(self.pose.y,4)
    
    def euclidean_distance(self,goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2) + pow((goal_pose.y - self.pose.y),2))
    
    def linear_vel(self,goal_pose,kp):
        return kp*(self.euclidean_distance(goal_pose))
    
    def heading_dirn(self,goal_pose):
        return atan2((goal_pose.y - self.pose.y),(goal_pose.x - self.pose.x))
    
    def angular_vel(self,goal_pose,kw):
        return kw*(self.heading_dirn(goal_pose) - self.pose.theta)
    
    def going_to_goal(self,x,y):
        goal_pose=Pose()

        goal_pose.x= x #1.42
        # For different gain values we will get some coordinates that causes mad behaviour
        goal_pose.y= y  #9
        error=0.001

        vel_msg=Twist()                                         # Twist Object
        while self.euclidean_distance(goal_pose)>=error:
            vel_msg.linear.x = self.linear_vel(goal_pose,1.5)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x=0
            vel_msg.angular.y=0
            vel_msg.angular.z=self.angular_vel(goal_pose,6)
            
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
        node=roslaunch.core.Node("turtlesim","turtlesim_node")
        launch=roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process=launch.launch(node)
        turtle=go_to_goal()
        print("************************************************************")
        turtle.going_to_goal(9.45,5.45)
        print("Reached first goal. \n", flush=True)
        print("************************************************************")
        turtle.going_to_goal(9.45,9.45)
        print("Reached second goal. \n", flush=True)
        print("************************************************************")
        turtle.going_to_goal(5.45,5.45)
        print("Reached third goal. \n", flush=True)
        print("************************************************************")
        turtle.orient_myway(1.57)
        print("Oriented as per my way \n",flush=True)
    except rospy.ROSInterruptException:
        pass       
        
