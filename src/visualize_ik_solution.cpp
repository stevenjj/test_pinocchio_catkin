// Package Path Definition
#include <Configuration.h>

// Include Punocchio Related Libraries
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
// Articulated Body Algorithm
#include "pinocchio/algorithm/aba.hpp"
// Composite Rigid Body Algorithm
#include "pinocchio/algorithm/crba.hpp"
// Recursive Newton-Euler Algorithm
#include "pinocchio/algorithm/rnea.hpp"

#include <math.h>       
// Import ROS and Rviz visualization
#include <ros/ros.h>
#include <bridge/val_rviz_translator.hpp>

class TestVal_IK{
public:
	TestVal_IK();
	~TestVal_IK();
    pinocchio::Model model;
    pinocchio::Data* data;

};

// Constructor
TestVal_IK::TestVal_IK(){
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_test.urdf";
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
	data = new pinocchio::Data(model);
}

// Destructor
TestVal_IK::~TestVal_IK(){
	delete data;
}

int main(int argc, char **argv){
	bool solved_ik_once = false;
	// Initialize ROS node for publishing joint messages
	ros::init(argc, argv, "test_rviz_ik_sol");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);

	// Transform broadcaster
	tf::TransformBroadcaster      br_ik;
	tf::TransformBroadcaster      br_robot;	
	// Joint State Publisher
	ros::Publisher robot_ik_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot1/joint_states", 10);
	ros::Publisher robot_joint_state_pub = n.advertise<sensor_msgs::JointState>("robot2/joint_states", 10);

	// Initialize Transforms and Messages
	tf::Transform tf_world_pelvis_init;
	tf::Transform tf_world_pelvis_end;

	sensor_msgs::JointState joint_msg_init; 
	sensor_msgs::JointState joint_msg_end; 

	// perform IK here
	TestVal_IK val_robot;

	while (ros::ok()){
	    // br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    // br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));

	    // robot_joint_state_pub.publish(joint_msg_init);
	    // robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;

}
