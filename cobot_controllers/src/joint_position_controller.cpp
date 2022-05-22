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
  
  n_ = ros::NodeHandle(node_handle);
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
  ROS_INFO_STREAM("init");
  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
      sub_joint_cmd_ = n_.subscribe<sensor_msgs::JointState>("/new_joint_target", 1, &JointPositionController::newJointTargetCallback, this);

      current_joint_values.reserve(7);
      goal_joint_values.reserve(7);

      for (size_t i = 0; i < 7; ++i) {
        current_joint_values[i] = position_joint_handles_[i].getPosition();
        goal_joint_values[i] = position_joint_handles_[i].getPosition();
        error_[i] = 0.0;
        error_decay_[i] = 0.0;
      }
      ROS_INFO_STREAM("Starting");
}

void JointPositionController::newJointTargetCallback(const sensor_msgs::JointState::ConstPtr& pose_msg)
{
  for (size_t i = 0; i < 7; ++i) {
      goal_joint_values[i] = pose_msg->position[i];
  }
  
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
      double k_p, k_d;
      k_p = 1.0;
      k_d = 0.75;
      for (size_t i = 0; i < 7; ++i)
      {
            current_joint_values[i] = position_joint_handles_[i].getPosition();
            if (error_[4] != 0.0){
              error_decay_[i] = (goal_joint_values[i] - current_joint_values[i]) - error_[i];
            }           
            error_[i] = goal_joint_values[i] - current_joint_values[i] ;

            double joint_i_command = k_p * error_[i] + k_d * error_decay_[i] ;
            position_joint_handles_[i].setCommand(current_joint_values[i] + joint_i_command*0.005);
            
      }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointPositionController,
                       controller_interface::ControllerBase)
