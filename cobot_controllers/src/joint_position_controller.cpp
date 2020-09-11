// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/joint_position_controller.h>

#include <cmath>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>

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
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {

      k_p = 2.0;  // damping ratio
      k_d = 5.0;  // natural frequency
      double T_d = k_p/k_d ;

      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      kinematic_model = robot_model_loader.getModel();
      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
      joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

      ROS_INFO("Starting");
      geometry_msgs::Pose pose ;
      pose.position.x = 0.46 ;
      pose.position.y = 0.01 ;
      pose.position.z = 0.20 ;

      Eigen::Isometry3d goal_pose_ ;
      tf::poseMsgToEigen(pose, goal_pose_);

      current_joint_values.reserve(7);
      goal_joint_values.reserve(7);

      for (size_t i = 0; i < 7; ++i) {
          current_joint_values[i] = position_joint_handles_[i].getPosition();
          ROS_INFO_STREAM("current_joint_values[" << i << "] = " << current_joint_values[i]);
      }

      robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
      ROS_INFO_STREAM("setJointGroupPositions");
      kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

      ROS_INFO_STREAM("Look for IK solution");
      double timeout = 0.1;
      found_ik = kinematic_state->setFromIK(joint_model_group, goal_pose_, timeout);
      if (found_ik)
      {
        ROS_INFO("Found IK solution");
        kinematic_state->copyJointGroupPositions(joint_model_group, goal_joint_values);
        for (size_t i = 0; i < 7; ++i)
        {
          error_[i] = goal_joint_values[i] - position_joint_handles_[i].getPosition() ;
        }
      }
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
      if (found_ik)
      {
        ROS_INFO("Updating");
        for (size_t i = 0; i < 7; ++i)
        {
          current_joint_values[i] = position_joint_handles_[i].getPosition();
          error_decay_[i] = (goal_joint_values[i] - current_joint_values[i]) - error_[i];
          error_[i] = goal_joint_values[i] - current_joint_values[i] ;
          double joint_i_command = k_p * (error_[i] + T_d*error_decay_[i]) ;
          position_joint_handles_[i].setCommand(current_joint_values[i] + joint_i_command*0.0001);
          ROS_INFO_STREAM("command[" << i << "] = " << current_joint_values[i] + joint_i_command);
        }

      }
      else
      {
        ROS_INFO("Did not find IK solution");
      }
}

}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointPositionController,
                       controller_interface::ControllerBase)
