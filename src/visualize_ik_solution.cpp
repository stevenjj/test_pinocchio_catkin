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

#define NUM_FLOATING_JOINTS 7 // 3 for x,y,z and 4 for qx, qy, qz, qw
#define JOINT_INDX_OFFSET 2 //pinocchio attaches a universe joint and a root joint that we need to remove

class TestVal_IK{
public:
	TestVal_IK();
	~TestVal_IK();
    pinocchio::Model model;
    pinocchio::Data* data;
    pinocchio::Data* data_ik;

    Eigen::VectorXd q_start;
    Eigen::VectorXd q_end;    

    int getJointId(const std::string & name);

};

// Constructor
TestVal_IK::TestVal_IK(){
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_test.urdf";
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
	data = new pinocchio::Data(model);


	// initialize configurations.
	q_start = Eigen::VectorXd::Zero(model.nq);
	q_end = Eigen::VectorXd::Zero(model.nq);

	// Initialize Identity Quaternion
	Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0);
	// Set Orientation
	q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w();

	q_end.segment<3>(4) = q_start.segment<3>(4);

	std::cout << "Joint Index of leftHipYaw: " << getJointId("leftHipYaw") << std::endl;
	std::cout << "Joint Index of rightHipYaw: " << getJointId("rightHipYaw") << std::endl;
	std::cout << "Joint Index of leftKneePitch: " << getJointId("leftKneePitch") << std::endl;

	q_start[2] = 1.0;

	q_start[getJointId("leftHipPitch")] = -0.3; 
	q_start[getJointId("rightHipPitch")] = -0.3;  
	q_start[getJointId("leftKneePitch")] = 0.6;  
	q_start[getJointId("rightKneePitch")] = 0.6;
	q_start[getJointId("leftAnklePitch")] = -0.3; 
	q_start[getJointId("rightAnklePitch")] = -0.3;

	q_start[getJointId("rightShoulderPitch")] = -0.2; 
	q_start[getJointId("rightShoulderRoll")] = 1.1;  
	q_start[getJointId("rightElbowPitch")] = 0.4;  
	q_start[getJointId("rightForearmYaw")] = 1.5; 

	q_start[getJointId("leftShoulderPitch")] = -0.2; 
	q_start[getJointId("leftShoulderRoll")] = -1.1;  
	q_start[getJointId("leftElbowPitch")] = -0.4;
	q_start[getJointId("leftForearmYaw")] = 1.5;  

	std::cout << "q_start:" << q_start.transpose() << std::endl;
	std::cout << "q_end:" << q_end.transpose() << std::endl;
}

int TestVal_IK::getJointId(const std::string & name){
	return NUM_FLOATING_JOINTS + model.getJointId(name) - JOINT_INDX_OFFSET;  
}

// Destructor
TestVal_IK::~TestVal_IK(){
	delete data;
	delete data_ik;
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

	// Initialize Rviz translator
	Val_Rviz_Translator rviz_translator;

	// perform IK here
	TestVal_IK val_robot;

	// Visualize in RVIZ
	rviz_translator.populate_joint_state_msg(val_robot.model, val_robot.q_start, tf_world_pelvis_init, joint_msg_init);
	rviz_translator.populate_joint_state_msg(val_robot.model, val_robot.q_end, tf_world_pelvis_end, joint_msg_end);

	while (ros::ok()){
	    br_robot.sendTransform(tf::StampedTransform(tf_world_pelvis_init, ros::Time::now(), "world",  "val_robot/pelvis"));
	    robot_joint_state_pub.publish(joint_msg_init);

	    br_ik.sendTransform(tf::StampedTransform(tf_world_pelvis_end, ros::Time::now(), "world", "val_ik_robot/pelvis"));
	    robot_ik_joint_state_pub.publish(joint_msg_end);
		ros::spinOnce();
		loop_rate.sleep();		
	}
	return 0;

}
