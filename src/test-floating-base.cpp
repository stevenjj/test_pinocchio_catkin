#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <math.h>       


#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"

// Articulated Body Algorithm
#include "pinocchio/algorithm/aba.hpp"
// Composite Rigid Body Algorithm
#include "pinocchio/algorithm/crba.hpp"
// Recursive Newton-Euler Algorithm
#include "pinocchio/algorithm/rnea.hpp"


int main(int argc, char ** argv)
{
  std::string filename = (argc<=1) ? "ur5.urdf" : argv[1];
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename, pinocchio::JointModelFreeFlyer(),model);
  pinocchio::Data data(model);

  // Eigen::VectorXd q = pinocchio::randomConfiguration(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  // floating base joints: x, y, z
  q[0] = 0.0;
  q[1] = 0.0;
  q[2] = 0.0;

  double theta = 0.0; //M_PI/4.0;
  std::cout << "pi = " << M_PI << std::endl;

  // floating base quaternion: qx, qy, qz, qw
  Eigen::AngleAxis<double> aa(theta, Eigen::Vector3d(0.0, 0.0, 1.0));
  Eigen::Quaternion<double> quat_init; quat_init =  aa;

  q[3] = quat_init.x();// 0.0;
  q[4] = quat_init.y(); //0.0;
  q[5] = quat_init.z(); //sin(theta/2.0);
  q[6] = quat_init.w(); //cos(theta/2.0);

  std::cout << "q = " << q.transpose() << std::endl;

  // Note that this function does not check if the quaternion is a valid unit quaternion.
  pinocchio::forwardKinematics(model,data,q);

  // List Joint SE(3) configurations after a forward kinematics
  // for (int k=0 ; k<model.njoints ; ++k){
  //   std::cout << model.names[k] << "\t: " << "translation:" << data.oMi[k].translation().transpose() << std::endl;
  //   std::cout << "rotation:\n" << data.oMi[k].rotation() << std::endl;
  // }

  std::cout << "size of q: " << q.size() << std::endl;
  std::cout << "num of joints: " << model.njoints << std::endl;


  // List joint names:
  // for (int k=0 ; k<model.njoints ; ++k){
  //   std::cout << "id:" << k << " " << model.names[k] <<  std::endl;
  // }  

  // List frames
  // std::cout << "number of frames in the model:" << model.frames.size() << std::endl;
  // for (int k=0 ; k<model.frames.size() ; ++k){
  //   std::cout << "frame:" << k << " " << model.frames[k].name <<  std::endl;
  // }


  // Get frame of the right Palm and compute its Jacobian
  pinocchio::FrameIndex rp_index = model.getFrameId("rightPalm");
  std::cout << "frame: " << model.frames[rp_index] << std::endl;

  pinocchio::Data::Matrix6x J_rpalm(6,model.nv); J_rpalm.fill(0);
  pinocchio::computeJointJacobians(model,data,q);
  pinocchio::updateFramePlacement(model, data, rp_index);
  pinocchio::getFrameJacobian(model,     data,        rp_index, pinocchio::WORLD, J_rpalm);

  std::cout << "J_rpalm w.r.t world:" << std::endl;
  std::cout << J_rpalm << std::endl;

  // Extract Jacobian blocks of interest
  // std::cout << "top 3 rows of J rpalm" << std::endl;
  // pinocchio::Data::Matrix3x J_rpalm_trans = J_rpalm.topRows<3>() ;
  // std::cout << J_rpalm_trans << std::endl;

  // std::cout << "bottom 3 rows of J rpalm" << std::endl;
  // pinocchio::Data::Matrix3x J_rpalm_rot = J_rpalm.bottomRows<3>() ;
  // std::cout << J_rpalm_rot << std::endl;



  // All the computation is stored in data. 

  // Get Inertia Matrix
  data.M.fill(0);  
  pinocchio::crba(model,data,q); // only computes the upper triangle part.
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  // std::cout << "Inertia matrix:" << std::endl;
  // std::cout << data.M << std::endl;

  // Get the Inverse of the Inertia Matrix
  pinocchio::computeMinverse(model,data,q); // only computes the upper triangle part
  data.Minv.triangularView<Eigen::StrictlyLower>() = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  // std::cout << "Inertia inverse:" << std::endl;
  // std::cout << data.Minv << std::endl;


  // Compute the coriolis matrix
  pinocchio::computeCoriolisMatrix(model,data,q,Eigen::VectorXd::Zero(model.nv));
  // std::cout << "Coriolis Matrix for 0 velocity:" << std::endl;
  // std::cout << data.C << std::endl;

  // Compute the gravity matrix
  pinocchio::computeGeneralizedGravity(model, data, q);
  // std::cout << "Gravity Matrix:" << std::endl;
  // std::cout << data.g << std::endl;


  // Example forward integration:
  Eigen::VectorXd vel(Eigen::VectorXd::Zero(model.nv));
  Eigen::VectorXd q_plus(model.nq);
  Eigen::VectorXd tau_plus(model.nv);
  const double dt = 1e-3;

  vel[0] = 0.5;  // move forward with 0.5 m/s
  vel[5] = M_PI/4.0; // rotate by M_PI/4 rad/s
  q_plus = pinocchio::integrate(model,q,vel*dt); // This performs a tangent space integration. Automatically resolves the quaternion components
  std::cout << "new configuration after a yaw rotation" << std::endl;
  std::cout << q_plus.transpose() << std::endl; 

  q = q_plus;
  vel[0] = 0.5; // move forward with 0.5 m/s
  vel[5] = 0.0;
  q_plus = pinocchio::integrate(model,q,vel*dt); // This performs a tangent space integration. Automatically resolves the quaternion components

  std::cout << "new configuration after a forward motion" << std::endl;
  std::cout << q_plus.transpose() << std::endl; 

  // To recompute the Jacobian:
/*
  // Do forward kinematics again
  pinocchio::forwardKinematics(model,data,q_plus);

  // Print Jacobian again
  pinocchio::computeJointJacobians(model,data,q_plus);
  pinocchio::updateFramePlacement(model, data, rp_index);
  pinocchio::getFrameJacobian(model,     data,        rp_index, pinocchio::WORLD, J_rpalm);

  std::cout << "J_rpalm w.r.t world:" << std::endl;
  std::cout << J_rpalm << std::endl;
*/

// Prepare to perform pseudo inverse
pinocchio::Data::RowMatrixXs J_task = J_rpalm;

const double svd_thresh = 1e-4;
unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_task.rows(), model.nv,svdOptions);
svd.setThreshold(svd_thresh);

Eigen::VectorXd dx_des = Eigen::VectorXd::Zero(6);
dx_des[0] = 1.0; // example error direction
Eigen::VectorXd dq = svd.compute(J_task).solve(dx_des);

std::cout << "dq change = " << std::endl;
std::cout << dq.transpose() << std::endl;


// std::cout << "number of rows in J_task: " <<  J_task.rows() << std::endl;
// pinocchio::Data::RowMatrixXs lambda = J_task*data.Minv*J_task.transpose()
// Answer with dynamically consistent inverse: 
// dq = data.Minv*J_task.transpose()*svd.compute(J_task*A*J_task.transpose()).solve(dx)



/*
// Prepare Jacobian
get(J)

// Compute M inverse
pinocchio::computeMinverse(model,data,q);

Store JMinvJ
JMinvJt = J*data.Minv*Jt

// Compute Cholesky Decomposition ... 
data.llt_JMinvJt.compute(data.JMinvJt);
*/




}
