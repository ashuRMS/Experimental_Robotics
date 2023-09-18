#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math


pub = [
    rospy.Publisher("/joint1_position/command", Float64, queue_size=10),
    rospy.Publisher("/joint2_position/command", Float64, queue_size=10),
    rospy.Publisher("/joint3_position/command", Float64, queue_size=10),
    rospy.Publisher("/joint4_position/command", Float64, queue_size=10)
]



def publish(arr):
    msg_list = []

    for i in range(4):
        msg = Float64()
        msg.data = arr[i]
        msg_list.append(msg)

    # Publish all joint positions simultaneously
    for i in range(4):
        pub[i].publish(msg_list[i])

if __name__ == "__main__":
    rospy.init_node("controller")

    while not rospy.is_shutdown():
        # Prompt the user for input
        input_str = input("Enter joint angles in radians (space-separated): ")

        # Split the input string into a list of strings
        float_str_list = input_str.split()

        # Check if the input has exactly 4 values
        if len(float_str_list) != 4:
            print("Please enter exactly 4 joint angles.")
            continue

        try:
            # Convert the list of strings to a list of floats
            joint_angles = [float(num) for num in float_str_list]
        except ValueError:
            print("Invalid input. Please enter valid numeric values.")
            continue

        publish(joint_angles)

    rospy.spin()


