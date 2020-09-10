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

  // cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }

  try {
    /*
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
    */
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  /*
  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }


  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianVelocityExampleController: Robot is not in the expected starting position "
            "for running this example. Run `roslaunch franka_example_controllers "
            "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
            "first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }
  */

  return true;
}

void CartesianVelocityController::starting(const ros::Time& /* time */) {
  k_p = 2.0 ;  // damping ratio
  k_d = 5.0 ;  // derivative time


  goal_pose_[0] = 0.46 ;
  goal_pose_[1] = 0.01 ;
  goal_pose_[2] = 0.15;

  current_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

  error_[0] = goal_pose_[0] - current_pose_[12];
  error_[1] = goal_pose_[1] - current_pose_[13];
  error_[2] = goal_pose_[2] - current_pose_[14];
}

void CartesianVelocityController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {

  current_pose_ = velocity_cartesian_handle_->getRobotState().O_T_EE_d;
  double T_d = k_p/k_d ;

  double error_decay_x = (goal_pose_[0] - current_pose_[12]) - error_[0] ;
  double error_decay_y = (goal_pose_[1] - current_pose_[13]) - error_[1] ;
  double error_decay_z = (goal_pose_[2] - current_pose_[14]) - error_[2] ;

  error_[0] = goal_pose_[0] - current_pose_[12] ;
  error_[1] = goal_pose_[1] - current_pose_[13];
  error_[2] = goal_pose_[2] - current_pose_[14];

  double x_d = k_p * (error_[0] + T_d*error_decay_x) ;
  double y_d = k_p * (error_[1] + T_d*error_decay_y) ;
  double z_d = k_p * (error_[2] + T_d*error_decay_z) ;

  double multiplier = 0.0075 ;
  x_d = x_d*multiplier ;
  y_d = y_d*multiplier ;
  z_d = z_d*multiplier ;

  ROS_INFO_STREAM("Sending command: " << " x_d=" << x_d << " y_d=" << y_d << " z_d=" << z_d) ;

  std::array<double, 6> command = {{ x_d, y_d, z_d, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::CartesianVelocityController, controller_interface::ControllerBase)
