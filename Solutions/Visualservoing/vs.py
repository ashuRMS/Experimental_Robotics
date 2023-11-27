#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VisualServoing:
    def __init__(self):
        self.reference_image_filename = "/home/ashutoshsahu/catkin_ws/src/beginner_tutorials/scripts/practical9_10/reference_image.png"
        self.reference_image = cv2.imread(self.reference_image_filename)
        self.bridge = CvBridge()
        self.error_fea=None
        self.feature_error_threshold = 200

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/odom", Odometry, self.pos_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Resize the current image to match the reference image size
            if self.reference_image.shape != current_image.shape:
                current_image = cv2.resize(current_image, (self.reference_image.shape[1], self.reference_image.shape[0]))

            # Calculate feature difference between current and reference images
            current_feature_vector = self.calculate_feature_vector(current_image)
            reference_feature_vector = self.calculate_feature_vector(self.reference_image)

            # # Display the current image
            # cv2.imshow("Current Image", current_image)
            
            # # Display the reference image
            # cv2.imshow("Reference Image", self.reference_image)

            # # Print or use the feature vectors as needed
            # print("Current Feature Vector:", current_feature_vector)
            # print("Reference Feature Vector:", reference_feature_vector)

            # calculate feature vector difference
            error_fea = current_feature_vector - reference_feature_vector
            

            # # Calculate and display the error
            error = np.linalg.norm(current_feature_vector - reference_feature_vector)
            print("Error:", error)

            # Calculate camera velocities
            camera_vel=self.camera_velocities(current_feature_vector, error_fea)

            # Publish camera velocities
            self.publish_velocities(camera_vel)

            # # Update the display
            # cv2.waitKey(1)



            # Check if the error is below the threshold
            if error < self.feature_error_threshold:
                # Stop the robot if the error is below the threshold
                self.stop_robot()


        except CvBridgeError as e:
            print(e)

    def pos_callback(self, msg):
        # Your position callback code goes here...
        pass

    def calculate_feature_vector(self, image):
        # Convert images to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the color ranges for each circle (adjust these values based on the actual colors)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create binary masks for each color
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Combine the masks
        combined_mask = mask_red + mask_green + mask_blue + mask_yellow

        # Find contours in the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize the feature vector
        feature_vector = []

        # Loop over the contours and extract centroid coordinates
        for contour in contours:
            # Calculate the moments of the contour
            M = cv2.moments(contour)

            # Avoid division by zero
            if M["m00"] != 0:
                # Calculate the centroid coordinates
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Append the centroid coordinates to the feature vector
                feature_vector.append(cx)
                feature_vector.append(cy)

        # Convert the feature vector to a NumPy array
        feature_vector = np.array(feature_vector)

        return feature_vector
    
    def camera_velocities(self,feature_vector,error_fea):
        lambda_f=825
        K=0.005
        depth=125
        interaction_matrix=np.array([[-lambda_f/depth, 0              ,   feature_vector[0]/depth],
                                     [ 0             , -lambda_f/depth,   feature_vector[1]/depth],
                                     [-lambda_f/depth, 0              ,   feature_vector[2]/depth],
                                     [ 0             , -lambda_f/depth,   feature_vector[3]/depth],
                                     [-lambda_f/depth, 0              ,   feature_vector[4]/depth],
                                     [ 0             , -lambda_f/depth,   feature_vector[5]/depth],
                                     [-lambda_f/depth, 0              ,   feature_vector[6]/depth],
                                     [ 0             , -lambda_f/depth,   feature_vector[7]/depth],])
        camera_vel = -K * np.dot(np.linalg.pinv(interaction_matrix), error_fea)

        R_yx = np.array([[0, 1, 0],
                        [0, 0, -1],
                        [1, 0, 0],
                        ])
        V_new = np.dot(R_yx, camera_vel)


        print("Camera Velocities:", V_new) 
        return V_new 
    
    def publish_velocities(self, camera_vel):
        twist_msg = Twist()
        twist_msg.linear.x = camera_vel[0]  # Set linear velocity in x direction
        twist_msg.linear.y = camera_vel[1]  # Set linear velocity in y direction
        twist_msg.angular.z = -camera_vel[2]  # Set angular velocity in z direction

        self.vel_pub.publish(twist_msg)


    def stop_robot(self):
        twist_msg = Twist()  # Create a Twist message
        twist_msg.linear.x = 0  # Set linear velocity in x direction
        twist_msg.linear.y = 0  # Set linear velocity in y direction
        twist_msg.angular.z = 0 # Set angular velocity in z direction

        self.vel_pub.publish(twist_msg)  # Publish the message to stop the robot
        print("-----------------------------------------")
        print("DESIRED POSE REACHED AS PER DEPTH SPECIFIED")
        print("-----------------------------------------")
        exit()

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('image_processing_node')

    # Create an instance of the VisualServoing class
    visual_servoing = VisualServoing()

    # Spin to keep the script running
    rospy.spin()
