#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,sqrt,atan2
import roslaunch

class go_to_goal:
    def __init__(self):
        rospy.init_node('turtle_contoller',anonymous=True)
        
        pub_topic="/turtle1/cmd_vel"
        sub_topic="/turtle1/pose"

        self.pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(sub_topic, Pose, self.update_pose)


        self.pose=Pose()
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
    
    def going_to_goal(self):
        goal_pose=Pose()
        x=int(input("Enter the desired x-coordinate :"))
        y=int(input("Enter the desired y-coordinate :"))

        error=0.5
        vel_msg=Twist()

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
        self.pub.publish(vel_msg)

        rospy.sleep(1.0)
        
if __name__=="__main__":
    try:
        node=roslaunch.core.Node("turtlesim","turtlesim_node")
        launch=roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process=launch.launch(node)
        turtle=go_to_goal()
        turtle.going_to_goal()
        
    except rospy.ROSInterruptException:
        pass       
        