// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <array>
#include <string>
#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>

#include <cobot_controllers/pseudo_inversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/server.h>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace cobot_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  std::array<double, 7> initial_pose_{};

  std::vector<double> current_joint_values;
  std::vector<double> goal_joint_values;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double last_tau[7], error_[7], error_decay_[7];
 

  // Subscribers
  ros::NodeHandle n_;
  ros::Subscriber sub_joint_cmd_ ;

  //Joint position callbacks
  void newJointTargetCallback(const sensor_msgs::JointState::ConstPtr& pose_msg) ;
};

}  // namespace franka_example_controllers