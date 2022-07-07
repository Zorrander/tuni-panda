// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/joint_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace cobot_controllers {

bool JointImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  n_ = ros::NodeHandle(node_handle);
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("JointImpedanceController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void JointImpedanceController::starting(const ros::Time& /*time*/) {
  sub_cmd_ = n_.subscribe<std_msgs::Float32MultiArray>("/new_target", 1, &JointImpedanceController::newTargetCallback, this);
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  pose_desired = initial_pose_;
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  k_p = 2.0;  // damping ratio
  k_d = 5.0;  // natural frequency
  double T_d = k_p/k_d ;
}


void JointImpedanceController::newTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

  ROS_INFO("MS");
  
  pose_desired = {array->data[0], array->data[1], array->data[2], array->data[3],
                    array->data[4], array->data[5], array->data[6], array->data[7],
                    array->data[8], array->data[9], array->data[10], array->data[11],
                    array->data[12], array->data[13], array->data[14], array->data[15] };
  
/*
  pose_desired = {array->data[0], array->data[4], array->data[8], array->data[12],
                    array->data[1], array->data[5], array->data[9], array->data[13],
                    array->data[2], array->data[6], array->data[10], array->data[14],
                    array->data[3], array->data[7], array->data[11], array->data[15] };
    
    */
}


/*void JointImpedanceController::newTargetCallback( const geometry_msgs::PoseStampedConstPtr& msg)
{

  std::array<double, 16> pose_desired = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  ROS_INFO_STREAM("newTargetCallback: \n");
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  ROS_INFO_STREAM("newTargetCallback: \n");
  kinematic_state->setVariablePositions(msg->pose);
  ROS_INFO_STREAM("setVariablePositions: \n");
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");
  // Print end-effector pose. Remember that this is in the model frame 
  ROS_INFO_STREAM("getGlobalLinkTransform: \n");
  Eigen::Vector3d position_d_ = end_effector_state.translation();
  Eigen::Matrix3d orientation_d_ = end_effector_state.rotation() ;
  ROS_INFO_STREAM("Translation: \n" << position_d_ << "\n");
  ROS_INFO_STREAM("Rotation: \n" << orientation_d_  << "\n");
  pose_desired[0] += orientation_d_(0,0);
  pose_desired[1] += orientation_d_(0,1);
  pose_desired[2] += orientation_d_(0,2);
  pose_desired[4] += orientation_d_(1,0);  
  pose_desired[5] += orientation_d_(1,1);
  pose_desired[6] += orientation_d_(1,2);
  pose_desired[8] += orientation_d_(2,0);
  pose_desired[9] += orientation_d_(2,1);
  pose_desired[10] += orientation_d_(2,2);
  pose_desired[12] += position_d_.x();
  pose_desired[13] += position_d_.y();
  pose_desired[14] += position_d_.z();
}*/


void JointImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  
  
  //current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  /*
  for (size_t i = 0; i < 16; ++i)
    {
      if (!(i==3 || i==7 || i==11 || i==15)) {
        //error_decay_[i] = (pose_desired[i] - current_pose[i]) - error_[i];
        //error_[i] = pose_desired[i] - current_pose[i] ;
        
        // pose_desired[i] = current_pose[i] + (k_p * (error_[i] + T_d*error_decay_[i]))*0.0005 ;
        pose_desired[i] = pose_desired[i]*0.1 ;
      }
    }*/
  
  for(int i=0;i<16;i++)
      ROS_INFO_STREAM( pose_desired[i]  << "\n");
   

  cartesian_pose_handle_->setCommand(pose_desired);

  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> JointImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace cobots_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointImpedanceController,
                       controller_interface::ControllerBase)
