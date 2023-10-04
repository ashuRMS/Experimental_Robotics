#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from beginner_tutorials.msg import Output_joint_positions

rospy.init_node("gazebo_controller",anonymous=True)

pub = [
    rospy.Publisher("/joint1_position/command", Float64, queue_size=10),
    rospy.Publisher("/joint2_position/command", Float64, queue_size=10),
    rospy.Publisher("/joint3_position/command", Float64, queue_size=10),
    rospy.Publisher("/joint4_position/command", Float64, queue_size=10)
]

joint_array=None

# Create a callback function
def topic_callback(data):
    global joint_array
    joint_array=data
    msg_list=[data.joint1_position,data.joint2_position,data.joint3_position,data.joint4_position]

    # Publish all joint positions simultaneously
    for i in range(4):
        pub[i].publish(msg_list[i])
    
# Create Publisher and Subscriber Node
rospy.Subscriber("Output_joint_position",Output_joint_positions,topic_callback)

if __name__ == "__main__":
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



