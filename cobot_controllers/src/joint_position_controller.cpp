// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cobot_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }


  sub_cmd_ = node_handle.subscribe<std_msgs::Float64MultiArray>("/new_target", 1, &JointPositionController::newTargetCallback, this);
  target_reached_pub = node_handle.advertise<std_msgs::Empty>("/target_reached", 1000);

  return true;
}

void JointPositionController::newTargetCallback(const std_msgs::Float64MultiArray::ConstPtr& pose_msg)
{
  ROS_INFO("Received target");

  for (size_t i = 0; i < 7; ++i) {
    goal_pose_[i] = pose_msg->data[i] ;
  }

  for (size_t i = 0; i < 7; ++i) {
    ROS_INFO_STREAM("goal_pose_[" << i << "] = " << goal_pose_[i]);
  }

  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionController::starting(const ros::Time& /* time */) {
  ROS_INFO("JointPositionController::starting");
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
    goal_pose_[i] = initial_pose_[i] ;
  }

  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  for (size_t i = 0; i < 7; ++i) {
    current_pose_[i] = position_joint_handles_[i].getPosition();
    err_[i] = goal_pose_[i]-current_pose_[i];
    // ROS_INFO_STREAM("err_[[" << i << "] = " << err_[i] );
  }

  // double angle = 1-exp(-elapsed_time_.toSec()/5);
  double angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / 5.0 ));
  for (size_t i = 0; i < 7; ++i) {
      if (err_[i] != 0.0){
        delta_[i] = err_[i] * std::sin(angle);
        position_joint_handles_[i].setCommand(current_pose_[i] + delta_[i]);
      }
  }
}


}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointPositionController,
                       controller_interface::ControllerBase)
