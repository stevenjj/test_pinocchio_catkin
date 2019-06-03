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

#define SVD_SOLVER JacobiSVD
//#define SVD_SOLVER BDCSVD

#include "pinocchio/utils/timer.hpp"


class TestVal_IK{
public:
	TestVal_IK();
	~TestVal_IK();
    pinocchio::Model model;

    std::unique_ptr<pinocchio::Data> data;
    std::unique_ptr<pinocchio::Data> data_ik;

    Eigen::VectorXd q_start;
    Eigen::VectorXd q_end;    
    Eigen::VectorXd dq_change;

    Eigen::Vector3d rfoot_des_pos;
    Eigen::Quaternion<double> rfoot_des_quat;    
    Eigen::Vector3d rfoot_pos_error;
	Eigen::Vector3d rfoot_ori_error;

	Eigen::Vector3d rfoot_cur_pos;
	Eigen::Quaternion<double> rfoot_cur_ori;
	Eigen::MatrixXd J_rfoot;

    Eigen::Vector3d lfoot_des_pos;
    Eigen::Quaternion<double> lfoot_des_quat;    
    Eigen::Vector3d lfoot_pos_error;
	Eigen::Vector3d lfoot_ori_error;

	Eigen::Vector3d lfoot_cur_pos;
	Eigen::Quaternion<double> lfoot_cur_ori;
	Eigen::MatrixXd J_lfoot;

    Eigen::MatrixXd J_task;    
    Eigen::VectorXd task_error;

	std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> > svd;
	unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

    void computeTranslationError(const Eigen::Vector3d & des, 
    							 const Eigen::Vector3d & current,
    							 Eigen::Vector3d & error);

    void computeQuaternionError(const Eigen::Quaternion<double> & des, 
    							const Eigen::Quaternion<double> & current,
    							Eigen::Vector3d & error);

    void getTaskJacobian(const std::string & frame_name, Eigen::MatrixXd & J);
    void getFrameWorldPose(const std::string & name, Eigen::Vector3d & pos, Eigen::Quaternion<double> & ori);

    int getJointId(const std::string & name);

   	void initialize_configuration();
    void initialize_desired();


    void printPose(const Eigen::Vector3d & pos, const Eigen::Quaternion<double> & ori);
    void printQuat(const Eigen::Quaternion<double> & ori);


    void updateKinematics(const Eigen::VectorXd & q_update);
	double doSingleStepIK();
	bool doFullIk(const int & max_iters);

private:
	// Pre-allocate some data types used
	pinocchio::FrameIndex tmp_frame_index;
	pinocchio::SE3 tmp_T_world;
	pinocchio::JointIndex tmp_joint_index;

	Eigen::AngleAxis<double> axis_angle;

    PinocchioTicToc timer;


	void doSmallTests();

};

// Constructor
TestVal_IK::TestVal_IK(){
	std::string filename = THIS_PACKAGE_PATH"models/valkyrie_test.urdf";
	pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
	data = std::unique_ptr<pinocchio::Data>(new pinocchio::Data(model));

	// Initialize timer. Use milliseconds
	timer = PinocchioTicToc(PinocchioTicToc::MS);

	initialize_configuration();
	initialize_desired();

	doSmallTests();
}

void TestVal_IK::doSmallTests(){
    Eigen::Vector3d cur_pos;
    Eigen::Quaternion<double> cur_ori;

	getFrameWorldPose("rightCOP_Frame", cur_pos, cur_ori);
	std::cout << "current Pose of rightCOP_Frame:" << std::endl; 
	printPose(cur_pos, cur_ori);

	getFrameWorldPose("leftCOP_Frame", cur_pos, cur_ori);
	std::cout << "current Pose of leftCOP_Frame:" << std::endl; 
	printPose(cur_pos, cur_ori);


/*
	Eigen::MatrixXd J_test(6, model.nv); J_test.fill(0); 
	getTaskJacobian("rightCOP_Frame",J_test);
	getTaskJacobian("leftCOP_Frame",J_test);

	// Initialize des Quaternion
	double theta = M_PI/4.0;
	Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
	Eigen::Quaternion<double> quat_des; quat_des =  aa;
	cur_ori = Eigen::AngleAxis<double>(-theta, Eigen::Vector3d(0.0, 0.0, 1.0));
	Eigen::Vector3d werror; werror.fill(0);
	computeQuaternionError(quat_des, cur_ori, werror);
*/
}

void TestVal_IK::initialize_configuration(){
	// initialize configurations.
	q_start = Eigen::VectorXd::Zero(model.nq);
	q_end = Eigen::VectorXd::Zero(model.nq);
	dq_change = Eigen::VectorXd::Zero(model.nv);

	// Initialize Floating base quaternion to have robot facing pi/4 to the left
	double theta = M_PI/4.0;
	Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));

	Eigen::Quaternion<double> init_quat(1.0, 0.0, 0.0, 0.0);
	init_quat = aa;

	// Set Orientation
	q_start[3] = init_quat.x(); q_start[4] = init_quat.y(); q_start[5] = init_quat.z(); q_start[6] = init_quat.w();

	q_end.segment<3>(4) = q_start.segment<3>(4);

	// std::cout << "Joint Index of leftHipYaw: " << getJointId("leftHipYaw") << std::endl;
	// std::cout << "Joint Index of rightHipYaw: " << getJointId("rightHipYaw") << std::endl;
	// std::cout << "Joint Index of leftKneePitch: " << getJointId("leftKneePitch") << std::endl;

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

	q_end = q_start;
	// Update configuration kinematics
	updateKinematics(q_start);
}


void TestVal_IK::updateKinematics(const Eigen::VectorXd & q_update){
	// Perform initial forward kinematics
	std::cout << "  ";	timer.tic();
	pinocchio::forwardKinematics(model, *data, q_update);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " forward kinematics computation time" << std::endl;

	// Compute Joint Jacobians
	std::cout << "  ";	timer.tic();
	computeJointJacobians(model,*data, q_update);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " jacobians computation time" << std::endl;

	// Update Frame Placements
	std::cout << "  ";	timer.tic();
    updateFramePlacements(model, *data);	
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " frame placements computation time" << std::endl;
}


void TestVal_IK::initialize_desired(){
	// Foot should be flat on the ground and spaced out by 0.25m on each side (0,0,0)    
    rfoot_des_pos.setZero();
    rfoot_des_pos[1] = -0.125;
	rfoot_des_quat.setIdentity();
	rfoot_pos_error.setZero();
	rfoot_ori_error.setZero();

	rfoot_cur_pos.setZero();
	rfoot_cur_ori.setIdentity();
	J_rfoot = Eigen::MatrixXd::Zero(6, model.nv);

    lfoot_des_pos.setZero();
    lfoot_des_pos[1] = 0.125;    
	lfoot_des_quat.setIdentity();
	lfoot_pos_error.setZero();    
	lfoot_ori_error.setZero();

	lfoot_cur_pos.setZero();
	lfoot_cur_ori.setIdentity();
	J_lfoot = Eigen::MatrixXd::Zero(6, model.nv);

	int task_dim = J_rfoot.rows() + J_lfoot.rows();
    J_task = Eigen::MatrixXd::Zero(task_dim, model.nv);    
    task_error = Eigen::VectorXd::Zero(task_dim);

  	std::cout << "Desired Pose of rightCOP_Frame:" << std::endl; 
  	printPose(rfoot_des_pos, rfoot_des_quat);

  	std::cout << "Desired Pose of rightCOP_Frame:" << std::endl; 
  	printPose(lfoot_des_pos, lfoot_des_quat);

    std::cout << "Task Dim:" << task_dim << std::endl;
    std::cout << "Task Error Dim:" << task_error.size() << std::endl;
    std::cout << "J_task rows, cols: " << J_task.rows() << " " << J_task.cols() << std::endl;

    // Initialize SVD. Allocate Memory.
    svd = std::unique_ptr< Eigen::SVD_SOLVER<Eigen::MatrixXd> >( new Eigen::SVD_SOLVER<Eigen::MatrixXd>(J_task.rows(), model.nv, svdOptions) );
    // Set Singular Value Threshold
	const double svd_thresh = 1e-4;
	svd->setThreshold(svd_thresh);
}


int TestVal_IK::getJointId(const std::string & name){
	return NUM_FLOATING_JOINTS + model.getJointId(name) - JOINT_INDX_OFFSET;  
}

void TestVal_IK::getFrameWorldPose(const std::string & name, Eigen::Vector3d & pos, Eigen::Quaternion<double> & ori){
  tmp_frame_index = model.getFrameId(name);
  // std::cout << "frame name: " << model.frames[tmp_frame_index] << std::endl;
  tmp_joint_index =  model.frames[tmp_frame_index].parent;
  // Return Data
  pos = data->oMf[tmp_frame_index].translation();
  ori = data->oMf[tmp_frame_index].rotation();

  // std::cout << "Operational Frame position w.r.t world:" << std::endl;
  // std::cout << data->oMf[tmp_frame_index] << std::endl;
  // std::cout << name << ":" << std::endl;
  // std::cout << "Pos 3D: " << pos.transpose() << std::endl;
  // std::cout << "Quat (x,y,z,w): " << ori.x() << " " <<
		// 							 ori.y() << " " <<
		// 							 ori.z() << " " <<
		// 							 ori.w() << " " <<
  // std::endl;
}

void TestVal_IK::getTaskJacobian(const std::string & frame_name, Eigen::MatrixXd & J){
  tmp_frame_index = model.getFrameId(frame_name);
  pinocchio::getFrameJacobian(model, *data, tmp_frame_index, pinocchio::WORLD, J);
  // std::cout << frame_name << " Jacobian: " << std::endl;
  // std::cout << J << std::endl;
}

void TestVal_IK::computeTranslationError(const Eigen::Vector3d & des, const Eigen::Vector3d & current, Eigen::Vector3d & error){
	error = des - current;
	// std::cout << "linear error:" << error.transpose() << std::endl;
}

void TestVal_IK::computeQuaternionError(const Eigen::Quaternion<double> & des, 
 		    							const Eigen::Quaternion<double> & current,
    									Eigen::Vector3d & error){
	// Perform quaternion error multiplication then cast it as an axis angle definition.
	axis_angle = des*current.inverse(); 
	// Convert to Vector3. error = w^hat_error * theta_error. 
	error = axis_angle.axis() * axis_angle.angle();

	// std::cout << "Error in axis Angle:" << std::endl;
	// std::cout << error.transpose() << std::endl;

}


void TestVal_IK::printPose(const Eigen::Vector3d & pos, const Eigen::Quaternion<double> & ori){
  std::cout << "Position: " << pos.transpose() << std::endl;
  printQuat(ori);
  std::cout << std::endl;
}

void TestVal_IK::printQuat(const Eigen::Quaternion<double> & ori){
  std::cout << "Quat (x,y,z,w): " << ori.x() << " " <<
									 ori.y() << " " <<
									 ori.z() << " " <<
									 ori.w() << " " <<
  std::endl;	
}

double TestVal_IK::doSingleStepIK(){
/*
	Update Kinematics

	get current pos and orientation of hand and feet.
	dx = des - current

	update the stacked task Jacobians
	J = [J_left^T, J_right_^T]^T

	compute the resulting command
	dq = Jinv * dx

	update the robot:
	q_plus = q + dq

	check error: if norm(dx) < epsilon break;
*/


	// Update Kinematics
	updateKinematics(q_end);

	// Get Current Position and Orientation 
	std::cout << "  ";	timer.tic();
    getFrameWorldPose("rightCOP_Frame", rfoot_cur_pos, rfoot_cur_ori);
    getFrameWorldPose("leftCOP_Frame", lfoot_cur_pos, lfoot_cur_ori);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " get pos+ori computation time" << std::endl;

    // Compute Linear and Orientation errors
    // dx = des - current;
	std::cout << "  ";	timer.tic();
    computeTranslationError(rfoot_des_pos, rfoot_cur_pos, rfoot_pos_error);
    computeTranslationError(lfoot_des_pos, lfoot_cur_pos, lfoot_pos_error);
    computeQuaternionError(rfoot_des_quat, rfoot_cur_ori, rfoot_ori_error);
    computeQuaternionError(lfoot_des_quat, lfoot_cur_ori, lfoot_ori_error);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) <<" error computation time" << std::endl;


    // Stack Errors
	std::cout << "  ";	timer.tic();
    task_error.head<3>() = rfoot_pos_error;
    task_error.segment<3>(3) = rfoot_ori_error;
    task_error.segment<3>(6) = lfoot_pos_error;
    task_error.segment<3>(9) = lfoot_ori_error;
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " dx stacking computation time" << std::endl;

    // std::cout << "task error = " << task_error.transpose() << std::endl;

    // Get Task Jacobians
	std::cout << "  ";	timer.tic();
	getTaskJacobian("rightCOP_Frame", J_rfoot);
	getTaskJacobian("leftCOP_Frame", J_lfoot);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " get Jacobians computation time" << std::endl;


	// Stack Jacobians
	std::cout << "  ";	timer.tic();
	J_task.topRows(6) = J_rfoot;
	J_task.bottomRows(6) = J_lfoot;
	// std::cout << "Task Jacobian:" << std::endl;
	// std::cout << J_task << std::endl;
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " Jacobian stacking computation time" << std::endl;


	// Compute dq = Jpinv*(dx)
	std::cout << "  ";	timer.tic();
	dq_change = svd->compute(J_task).solve(task_error);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " pinv(J)*dx computation time" << std::endl;

	// std::cout << "dq_change:" << std::endl;
	// std::cout << dq_change.transpose() << std::endl;

	// Update once:
	// q = q + dq;
	std::cout << "  ";	timer.tic();
	q_end = pinocchio::integrate(model, q_end, dq_change);
	std::cout << timer.toc() << timer.unitName(timer.DEFAULT_UNIT) << " q = q+dq computation time" << std::endl;


    // std::cout << "error norm = " << task_error.norm() << std::endl;

    // Return the error norm
    return task_error.norm();

}

bool TestVal_IK::doFullIk(const int & max_iters){
	double ik_error_norm = 1000.0;
	double eps = 1e-6;
	// Create a timer that outputs microseconds
	for(int i = 0; i < max_iters; i++){
		std::cout << "iter: " << i+1 << std::endl;
		ik_error_norm = doSingleStepIK();

		std::cout << "   error_norm = " << ik_error_norm << std::endl;

		if (ik_error_norm <= eps){
			std::cout << "Final error norm = " << ik_error_norm << std::endl;
			std::cout << "Converged to within error norm = " << eps << " after " << i+1 << " iters" <<std::endl;
			std::cout << "q_end:" << q_end.transpose() << std::endl;
			return true;
			break;
		}

	}

	if (ik_error_norm > eps){
			std::cout << "Final error norm = " << ik_error_norm << std::endl;
			std::cout << "Failed to converge to within error norm = " << eps << " after " << max_iters << " iters" << std::endl;
			std::cout << "q_end:" << q_end.transpose() << std::endl;
			return false;
	}
}



// Destructor
TestVal_IK::~TestVal_IK(){
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
	val_robot.doFullIk(10); // Do a maximum of 10 iterations of IK descent

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
