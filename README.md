# Experimental Robotics
This repository contains the hardware and software implementation of my learning on robotics. It has broadly 6 practical experiments. They involve the following.
* Introduction to ROS.
* Control of Turtlebot3.
* Control of Open Manipulator X.
* End effector and trajectory control of Open Manipulator X.
* Jacobian-based control of the end effector of Open Manipulator X.

# Introduction to ROS
## Task A: Run TurtleSim using linear and angular velocities as the inputs.
* Open 2 Linux terminals. Run the following commands each in a separate terminal   

`$ roscore ` <br>
`$ rosrun turtlesim turtlesim_node`


* Now inside the new terminal write the following command <br>
`$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0 , 0.0 , 0.0]' '[0.0 , 0.0, 1.5]' ` <br>

This will publish the velocity commands at a rate of 1 Hz. The linear velocity is 2m/sec, in the x- direction. And the angular velocity is 1.5 rad/sec.

## Task B: Write a custom code to send the TurtleSim to the desired position.
[Desired position](./Solutions/p1_B.py)

## Task C: Write a custom service and client node, where the client requests basic calculations from the server
[Client](./Solutions/p1_C_client.py)
[Server](./Solutions/p1_C_server.py)

## Task D: Create a ROS package by your name without using the "catkin_create_pkg" command
[Link to documentation](https://wiki.ros.org/ROS/Tutorials/Creating%20a%20Package%20by%20Hand) <br>
[CMakeLists.txt](./Solutions/p1_D_CMakeLists.txt) <br>
[package.xml](./Solutions/p1_D_package.xml) <br>



