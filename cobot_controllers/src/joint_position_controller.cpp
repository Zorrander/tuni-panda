// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/joint_position_controller.h>
#define _USE_MATH_DEFINES
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

  n_ = ros::NodeHandle(node_handle);
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  // state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
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

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {

      sub_cmd_ = n_.subscribe<geometry_msgs::Pose>("/new_target", 1, &JointPositionController::newTargetCallback, this);
      target_reached_pub = n_.advertise<std_msgs::Empty>("/target_reached", 1000);


      k_p = 100.0;  // damping ratio
      k_d = 20.0;  // natural frequency
      double T_d = k_p/k_d ;

      current_joint_values.reserve(7);
      current_command_.reserve(3);

      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      kinematic_model = robot_model_loader.getModel();
      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
      joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
      found_ik = false;
      target_reached_ = true;
      ROS_INFO("JointPositionController::starting");
}

void JointPositionController::newTargetCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
  // Eigen::Isometry3d goal_pose_ ;
  // tf::poseMsgToEigen(pose, goal_pose_);
  ROS_INFO("Received target");
  goal_joint_values.reserve(7);

  for (size_t i = 0; i < 7; ++i) {
      current_joint_values[i] = position_joint_handles_[i].getPosition();
      ROS_INFO_STREAM("current_joint_values[" << i << "] = " << current_joint_values[i]);
  }

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

  double timeout = 0.1;
  found_ik = kinematic_state->setFromIK(joint_model_group, *pose_msg, timeout);
  if (found_ik)
  {
    ROS_INFO("Found IK solution");
    kinematic_state->copyJointGroupPositions(joint_model_group, goal_joint_values);
    for (size_t i = 0; i < 7; ++i)
    {
      error_[i] = goal_joint_values[i] - position_joint_handles_[i].getPosition() ;
    }
  }
  else {
    ROS_INFO("Did not find IK solution");
  }
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
      if (found_ik)
      {
        //state_interface_handle_ = state_interface->getHandle("panda_arm_robot");
        //robot_state = state_interface_handle_.getRobotState();

        // get state variables
        //coriolis_array = model.coriolis(robot_state);
        //jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        // convert to Eigen
        //coriolis(coriolis_array.data());
        //jacobian(jacobian_array.data());

        target_reached_ = true;
        for (size_t i = 0; i < 7; ++i)
        {
          // Compute error and error rate evolution
          current_joint_values[i] = position_joint_handles_[i].getPosition();
          error_decay_[i] = (goal_joint_values[i] - current_joint_values[i]) - error_[i];
          error_[i] = goal_joint_values[i] - current_joint_values[i] ;

          //Detect if the target has been reached or not
          if (error_[i] > 0.01){
            target_reached_ = false;
          }
        }


        // Send signal if the target has been reached
        if (target_reached_){
          ROS_INFO("Target reached");
          for (size_t i = 0; i < 7; ++i)
          {
            current_joint_values[i] = position_joint_handles_[i].getPosition();
            position_joint_handles_[i].setCommand(current_joint_values[i]);
          }
          std_msgs::Empty msg ;
          target_reached_pub.publish(msg);


        } else {
          for (size_t i = 0; i < 7; ++i)
          {
            // Commands - PD control
            double joint_i_command = k_p * (error_[i] + T_d*error_decay_[i]) ;
            position_joint_handles_[i].setCommand(current_joint_values[i] + joint_i_command*0.0005);
          }
        }

      }
      else
      {
        for (size_t i = 0; i < 7; ++i)
        {
          current_joint_values[i] = position_joint_handles_[i].getPosition();
          position_joint_handles_[i].setCommand(current_joint_values[i]);
        }
      }
}

}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointPositionController,
                       controller_interface::ControllerBase)
