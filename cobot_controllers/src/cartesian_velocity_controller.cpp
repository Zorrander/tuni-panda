// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cobot_controllers {

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }

  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianVelocityController::starting(const ros::Time& /* time */) {
  goal_pose_[0] = 0.46 ;
  goal_pose_[1] = 0.01 ;
  goal_pose_[2] = 0.15;
}

void CartesianVelocityController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {

  elapsed_time_ += period;
  double angle, delta_x, delta_y, delta_z, radius;

  current_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

  // angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  angle = 1 - exp(-1 * elapsed_time_.toSec()/10);

  ROS_INFO_STREAM(" error_x= " << goal_pose_[0]-current_pose_[12]) ;
  ROS_INFO_STREAM(" error_x= " << goal_pose_[1]-current_pose_[13]) ;
  ROS_INFO_STREAM(" error_x= " << goal_pose_[2]-current_pose_[14]) ;

  delta_x = (goal_pose_[0]-current_pose_[12]) * angle;
  delta_y = (goal_pose_[1]-current_pose_[13]) * angle;
  delta_z = (goal_pose_[2]-current_pose_[14]) * angle;

  ROS_INFO_STREAM(" delta_x= " << delta_x) ;
  ROS_INFO_STREAM(" delta_y= " << delta_y) ;
  ROS_INFO_STREAM(" delta_z= "<<  delta_z) ;

  ROS_INFO_STREAM("Sending command: " << " x_d=" << delta_x << " y_d=" << delta_y << " z_d=" << delta_z) ;

  std::array<double, 6> command = {{ current_pose_[12]+delta_x, current_pose_[13]+delta_y, current_pose_[14]+delta_z, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::CartesianVelocityController, controller_interface::ControllerBase)
