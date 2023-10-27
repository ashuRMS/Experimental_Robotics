// 1.) include headers files 
#include <ros/ros.h>
#include <beginner_tutorials/Joint_position.h> // include custom messages
#include <beginner_tutorials/End_effector_position.h>
#include <cmath>
#include <Eigen/Dense> // c++ library for linear algebra

// 2.) Declare global variable of type beginner_tutorials::Joint_position 
beginner_tutorials::Joint_position joint_array;

// 3.) Publisher and Subscriber Declarations
ros::Publisher endEffectorPub;
ros::Subscriber jointSub;

// 4.) Forward Kinematics
void forwardKinematics(const beginner_tutorials::Joint_position& joint_array) // read only variable
{
    // Load joint angles
    double theta1=joint_array.joint1_position;
    double theta2=joint_array.joint2_position;
    double theta3=joint_array.joint3_position;
    double theta4=joint_array.joint4_position;


   // D-H parameters
    double theta0=0.1853;
    int n=4;
    Eigen::VectorXd alp(4);
    Eigen::VectorXd a(4);
    Eigen::VectorXd d(4);
    Eigen::VectorXd theta(4);
    Eigen::VectorXd p4E_4(3);

    // Initialize transformation matrices
    Eigen::Matrix4d Ttemp = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d HTM[4];

    // Initialize D-H parameters
    alp << 0, M_PI / 2, 0, 0;
    a << 0.012, 0, 0.130, 0.124;
    d << 0.077, 0, 0, 0;
    theta << theta1, -theta2 + M_PI / 2 - theta0, -theta3 - M_PI / 2 + 0.1853, -theta4;
    p4E_4 << 0.126, 0, 0;

    for (int i = 0; i < n; ++i) {
        double t11 = cos(theta[i]);
        double t12 = -sin(theta[i]);
        double t13 = 0;
        double t14 = a[i];

        double t21 = sin(theta[i]) * cos(alp[i]);
        double t22 = cos(theta[i]) * cos(alp[i]);
        double t23 = -sin(alp[i]);
        double t24 = -d[i] * sin(alp[i]);

        double t31 = sin(theta[i]) * sin(alp[i]);
        double t32 = cos(theta[i]) * sin(alp[i]);
        double t33 = cos(alp[i]);
        double t34 = d[i] * cos(alp[i]);


        Eigen::Matrix4d Tiim1;
        Tiim1 << t11, t12, t13, t14,
                 t21, t22, t23, t24,
                 t31, t32, t33, t34,
                 0, 0, 0, 1;

        Eigen::Matrix4d Ti0 = Ttemp * Tiim1;
        HTM[i] = Ti0;
        Ttemp = Ti0;
    
    }
    // Homogeneous Transformation matrix
    Eigen::Matrix4d Tn0 = HTM[n - 1];

    // End Effector's position
    Eigen::Vector4d p4E_4_h;
    p4E_4_h << p4E_4[0], p4E_4[1], p4E_4[2], 1;
    Eigen::Vector4d pE0_h = Tn0 * p4E_4_h;
    Eigen::Vector3d pE0 = pE0_h.head(3);

    // For publishing on custom topic
    beginner_tutorials::End_effector_position pose;
    pose.x = pE0[0];
    pose.y = pE0[1];
    pose.z = pE0[2];
    endEffectorPub.publish(pose);
}

// 5.) create callback function
void topicCallback(const beginner_tutorials::Joint_position::ConstPtr& data)
{
    //beginner_tutorials::Joint_position::ConstPtr: This is a data type. 
    //It represents a shared pointer to an instance of the beginner_tutorials::Joint_position 
    //message type. In ROS, messages are typically passed using shared pointers t
    //to manage memory efficiently.
    joint_array=*data;
    forwardKinematics(joint_array);
}


int main (int argc, char** argv)
{
    ros::init(argc,argv,"forward_kinematics");
    ros::NodeHandle nh;

    jointSub = nh.subscribe("Joint_position",1, topicCallback);
    endEffectorPub = nh.advertise<beginner_tutorials::End_effector_position>("End_effector_position",10);
    // This line sets up a ROS publisher. It publishes messages of type beginner_tutorials::End_effector_position 
    //on the "End_effector_position" topic.
    ros::spin();
    return 0;
}
