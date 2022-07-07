
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/cartesian_pose_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cobot_controllers {

bool CartesianPoseController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  n_ = ros::NodeHandle(node_handle);
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("/franka_control/arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  sub_cmd_ = node_handle.subscribe<std_msgs::Float32MultiArray>("ee_target", 10, &CartesianPoseController::commandCB, this);

  return true;
}

void CartesianPoseController::commandCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    broadcast_started=true;
    int i = 0;
    std::cout << "Received";
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
            Arr[i] = *it;
            i++;  
            std::cout << Arr[i] << "\n";
    }
}

void CartesianPoseController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
  broadcast_started = false;
  std::cout << initial_pose_[12] << "\n"<< initial_pose_[13]<< "\n"<< initial_pose_[14] << "\n"<< initial_pose_[15]<< "\n";
}

void CartesianPoseController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  std::array<double, 16> new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  
  if (broadcast_started==true){

    //new_pose[0] = Arr[3];
    //new_pose[1] = Arr[4];
    //new_pose[2] = Arr[5];

    //new_pose[4] = Arr[6];
    //new_pose[5] = Arr[7];
    //new_pose[6] = Arr[8];

    //new_pose[8] = Arr[9];
    //new_pose[9] = Arr[10];
    //new_pose[10] = Arr[11];

    new_pose[12] =  Arr[0]; //- Arr[0])*0.001;
    new_pose[13] =  Arr[1]; //- Arr[1])*0.001;
    new_pose[14] =  Arr[2]; //- Arr[2])*0.001;
    std::cout << "Moving to";
    std::cout << new_pose[12];
    std::cout << new_pose[13];
    std::cout << new_pose[14];
  } else {
    new_pose = initial_pose_;
  }

  cartesian_pose_handle_->setCommand(new_pose);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
