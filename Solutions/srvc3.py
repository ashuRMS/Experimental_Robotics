#!/usr/bin/env python3

########################## THIS WORKS FINE #################################

# Import necessary ROS libraries and message types
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from beginner_tutorials.msg import Joint_position,Output_joint_positions

# Define a class for the JointPositionControlNode
class JointPositionControlNode:
    def __init__(self):
        # Initialize the ROS node with a unique name and make it anonymous
        rospy.init_node("joint_position_control_node", anonymous=True)
        
        # Name of the service to control the robot's joint positions
        self.service_name = 'goal_joint_space_path'
        
        # Setup the subscriber to listen for joint position messages
        self.setup_subscriber()
        
        # Keep the ROS node running
        rospy.spin()

    # Method to set up the subscriber for receiving joint position messages
    def setup_subscriber(self):
        # Subscribe to the "Joint_position" topic with message type Joint_position  --fk
        rospy.Subscriber("Joint_position", Joint_position, self.callback)

        # Subscribe to the "Output_joint_positions" topic with message type Joint_position  --ik
        #rospy.Subscriber("Joint_position", Output_joint_positions, self.callback)

    # Callback function that gets executed when a joint position message is received
    def callback(self, data):
        # Extract joint positions from the received message
        j1 = data.joint1_position
        j2 = data.joint2_position
        j3 = data.joint3_position
        j4 = data.joint4_position
        
        # Call the set_position method to control the robot with these joint positions
        self.set_position(j1, j2, j3, j4)

    # Method to send joint positions to the robot control service
    def set_position(self, j1, j2, j3, j4):
        # Log a message indicating that we are waiting for the service
        rospy.loginfo("Waiting for service %s...", self.service_name)
        
        try:
            # Wait for the specified service to become available with a timeout
            rospy.wait_for_service(self.service_name, timeout=10.0)
            
            # Create a service proxy to make requests to the service
            set_position = rospy.ServiceProxy(self.service_name, SetJointPosition)
            
            # Create a request message and populate it with joint position data
            arg = SetJointPositionRequest()
            arg.planning_group = "Open Manipulator X Joint Position Control"  # Replace with the appropriate planning group
            arg.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]  # Replace with actual joint names
            arg.joint_position.position = [j1, j2, j3, j4]
            arg.path_time = 4.0  # Specify the desired path time
            
            # Send the request to the service and store the response
            resp1 = set_position(arg)
            
            # Log a message indicating that the service call is done
            rospy.loginfo('Service done!')
        
        except rospy.ServiceException as e:
            # Log an error message if the service call fails
            rospy.logerr("Service call failed: %s", e)

# Entry point of the script
if __name__ == "__main__":
    try:
        # Create an instance of the JointPositionControlNode class
        joint_position_node = JointPositionControlNode()
    
    except rospy.ROSInterruptException:
        # Log a warning if the ROS node is interrupted
        rospy.logwarn("ROS node interrupted.")
