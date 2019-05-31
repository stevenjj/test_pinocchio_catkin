#ifndef VAL_RVIZ_TRANSLATOR_H
#define VAL_RVIZ_TRANSLATOR_H

#include <Eigen/Dense>
#include "pinocchio/multibody/model.hpp"

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

class Val_Rviz_Translator{
public:
  void populate_joint_state_msg( const pinocchio::Model & model,
  								 const Eigen::VectorXd & q,
                                 tf::Transform & world_to_pelvis_transform,
                                 sensor_msgs::JointState & joint_state_msg);
  int getJointId(const pinocchio::Model & model, const std::string & name);

  Val_Rviz_Translator();
  ~Val_Rviz_Translator();  


};

#endif