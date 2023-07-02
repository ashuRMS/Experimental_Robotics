# Experimental Robotics
This repository contains the hardware and software implementation of my learning on robotics. It has broadly 6 practical experiments. They involve the following.
* Introduction to ROS.
* Control of Turtlebot3.
* Control of Open Manipulator X.
* End effector and trajectory control of Open Manipulator X.
* Jacobian-based control of the end effector of Open Manipulator X.

# Introduction to ROS
## Task A: Run TurtleSim using linear and angular velocities as the inputs.
* Open 2 Linux terminals. Run the following commands each in separate terminal   

`$ roscore ` <br>
`$ rosrun turtlesim turtlesim_node`


* Now inside new terminal write the following command <br>
`$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0 , 0.0 , 0.0]' '[0.0 , 0.0, 1.5]' ` <br>

This will publish the velocity commands at a rate of 1 Hz. The linear velocity is 2m/sec, in x- direction. And the angular velocity is 1.5 rad/sec.

## Task B: Write a custom code to send the TurtleSim to the desired position.
[Click here to view the file.py](./Solutions/file.py)




