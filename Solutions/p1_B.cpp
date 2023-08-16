#include <iostream>
#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "cmath"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TurtleController {

private:
    ros::NodeHandle* nh;            // Pointer to ros::NodeHandle object for creating ROS nodes
    ros::Publisher pub;             // Create a publisher object
    ros::Subscriber sub;            // Create a subscriber object
    std::string pub_topic;          // String variable to store name of topic on which we publish
    std::string sub_topic;          // String variable to store name of topic to which we subscribe
    turtlesim::Pose pose;           // Create pose object
    ros::Rate* rate;                // pointer to rate object to control rate of ROS node's main loop iterations (int main)
    double kp,kw;

public:
    TurtleController(int argc,char** argv) {
        // Initialize ROS node
        ros::init(argc, argv, "turtle_controller_cpp");
        nh = new ros::NodeHandle();

        // Publisher and subscriber topics
        pub_topic = "/turtle1/cmd_vel";
        sub_topic = "/turtle1/pose";

        // Create publisher and subscriber
        pub = nh->advertise<geometry_msgs::Twist>(pub_topic, 10);
        sub = nh->subscribe(sub_topic, 10, &TurtleController::updatePose, this);

        // Initialize pose object and rate
        pose = turtlesim::Pose();
        rate = new ros::Rate(100);
    }

    // Callback to update the pose
    void updatePose(const turtlesim::Pose::ConstPtr& msg) {
        // Update pose object using msg parameter
        pose.x = msg->x;
        pose.y = msg->y;
        pose.theta =msg->theta;
        
    }

    //  Calculate the euclidean distance between desired and current position
    double euclideanDistance(const turtlesim::Pose& goalPose){
        return std::sqrt(std::pow(goalPose.x - pose.x,2) + std::pow(goalPose.y - pose.y,2));
    }

    // Calculate linear velocity based on euclidean distance. Kp is gain parameter
    double linearVel(const turtlesim::Pose& goalPose,double kp){
        return kp*(euclideanDistance(goalPose));
    }

    // Calculate the heading direction
    double headingdirn(const turtlesim::Pose& goalPose){
        return std::atan2(goalPose.y - pose.y,goalPose.x - pose.x);
    }

    // Create quaternions from angle1 and angle2
    double calculateAngleDifference(double angle2, double angle1) {
        tf2::Quaternion q1;
        q1.setRPY(0, 0, angle1);
        
        tf2::Quaternion q2;
        q2.setRPY(0, 0, angle2);
        
        // Calculate quaternion difference
        tf2::Quaternion q_diff = q2 * q1.inverse();
        
        // Convert quaternion difference back to angles
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_diff).getRPY(roll, pitch, yaw);

        double angle_diff = yaw;
        
    return angle_diff;
    }

    // Calculates the angular velocities
    double angularVel(const turtlesim::Pose& goalPose,double kw){
        return kw*(calculateAngleDifference(headingdirn(goalPose),pose.theta));
    }

    // Move until we are there
    void going_to_goal(double x,double y){
        turtlesim::Pose goalPose;

        goalPose.x=x;
        goalPose.y=y;

        pose.x=5.5444;
        pose.y=5.5444;
        
        double error = 0.05;

        //Using geometry_msgs::Twist for Twist Object
        geometry_msgs::Twist velMsg; 
        while (euclideanDistance(goalPose)>=0.01){
            // std::cout<<"x"<<pose.x<<std::endl;
            // std::cout<<"y"<<pose.y<<std::endl;
            velMsg.linear.x = linearVel(goalPose,1.5);
            velMsg.linear.y = 0;
            velMsg.linear.z = 0;

            velMsg.angular.x = 0;
            velMsg.angular.y = 0;
            velMsg.angular.z = angularVel(goalPose,6.0);

            pub.publish(velMsg);
            ros::spinOnce();  // process callbacks for subscribers
            rate->sleep();
            pose=pose;  // update the pose 
        }

        // stop the robot
        velMsg.linear.x=0;
        velMsg.angular.z=0;
        pub.publish(velMsg);
        
        ROS_INFO_STREAM("Euclidean distance Error: " << euclideanDistance(goalPose) << ", (X,Y) = (" << pose.x <<","<< pose.y << ")");
    
    }
    void orient_myway(double goal_theta){
        //This function orients the turtle in desired orientation which was not possible with going_to_goal function 
        turtlesim::Pose goalPose;
        goalPose.theta = goal_theta;

        geometry_msgs::Twist velMsg;
        while(goalPose.theta - pose.theta >=0.1){
            velMsg.angular.z = 1*(goalPose.theta - pose.theta);
            pub.publish(velMsg);
            ros::spinOnce();   /// a very important command
            rate->sleep();
            pose=pose;
        }
        // stop the robot
        velMsg.linear.x=0;
        velMsg.angular.z=0;
        pub.publish(velMsg);
        
        ROS_INFO_STREAM("Heading Error:" <<goalPose.theta - pose.theta);   
    }

};

int main(int argc, char** argv) {

        float x,y;

        std::cout << "Enter the x-coordinate in range (0, 11): ";
        std::cin >> x;
        std::cout << "Enter the y-coordinate in range (0, 11): ";
        std::cin >> y;
        ros::init(argc,argv,"turtle_controller_cpp");
    

    try{
        ros::NodeHandle nh;
        TurtleController turtle(argc,argv);
        std::cout << "************************************************************" << std::endl;
        turtle.going_to_goal(x,y);
        std::cout<< " Reached first goal."<<std::endl;
        std::cout << "************************************************************" << std::endl;
        // turtle.going_to_goal(5.45,9.45);
        // std::cout<< " Reached second goal."<<std::endl;
        // std::cout << "************************************************************" << std::endl;
        // turtle.going_to_goal(5.45,5.45);
        // std::cout<< " Reached third goal."<<std::endl;
        // std::cout << "************************************************************" << std::endl;
        // turtle.orient_myway(1.57);
        // std::cout<< " Oriented as per my way"<<std::endl;
        // std::cout << "************************************************************" << std::endl;

        ros::spin();
        return 0;
    } catch (const ros::Exception& e ){
        ROS_INFO("ROS Interrupt Exception: %s", e.what());

    }
    return 0;
}




