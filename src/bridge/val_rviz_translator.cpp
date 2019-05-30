#include <bridge/val_rviz_translator.hpp>

// Constructor
Val_Rviz_Translator::Val_Rviz_Translator(){}
// Destructor
Val_Rviz_Translator::~Val_Rviz_Translator(){}

void Val_Rviz_Translator::populate_joint_state_msg(const pinocchio::Model & model,
                                                   const Eigen::VectorXd & q, 
                                                    tf::Transform & world_to_pelvis_transform,
                                                    sensor_msgs::JointState & joint_state_msg){
  // Prepare the joint state message
  sensor_msgs::JointState joint_states;
  std::string joint_name;
  double joint_value = 0.0;  

  // for (size_t i = 0; i < (valkyrie::num_qdot - valkyrie::num_virtual); i++){
  //   joint_name = valkyrie_joint::names[i];
  //   joint_value = q[i + valkyrie::num_virtual];

  //   joint_states.name.push_back(joint_name);
  //   joint_states.position.push_back(joint_value);
  // }


  // Prepare the world to pelvis transform
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(q[0], q[1], q[2]) ); //x, y, z
  // Convert so3_pelvis to quaternion
  // sejong::Vect3 so3_pelvis; so3_pelvis.setZero(); 
  // so3_pelvis[0] = q[3]; so3_pelvis[1] = q[4]; so3_pelvis[2] = q[5];
  // sejong::Quaternion quat_pelvis;
  // sejong::convert(so3_pelvis, quat_pelvis);

  // // Initialize quaternion and set the transform
  // tf::Quaternion quat_world_to_pelvis(quat_pelvis.x(), quat_pelvis.y(), quat_pelvis.z(), quat_pelvis.w()); //x, y, z, w
  // transform.setRotation(quat_world_to_pelvis);

  // Set Output
  joint_state_msg = joint_states;
  world_to_pelvis_transform = transform;
}
