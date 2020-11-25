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
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseController: Could not get state interface from hardware");
    return false;
  }

  sub_cmd_ = node_handle.subscribe<geometry_msgs::Pose>("/new_target", 1, &CartesianPoseController::newTargetCallback, this);
  target_reached_pub = node_handle.advertise<std_msgs::Empty>("/target_reached", 1000);

  return true;
}

void CartesianPoseController::newTargetCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
  ROS_INFO("Received target");

  ROS_INFO_STREAM("pose_msg->position.x = " << pose_msg->position.x);
  ROS_INFO_STREAM("pose_msg->position.x = " << pose_msg->position.y);
  ROS_INFO_STREAM("pose_msg->position.x = " << pose_msg->position.z);

  goal_pose_[0] = pose_msg->position.x ;
  goal_pose_[1] = pose_msg->position.y ;
  goal_pose_[2] = pose_msg->position.z ;

  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseController::starting(const ros::Time& /* time */) {
  ROS_INFO("CartesianPoseController starting");
  elapsed_time_ = ros::Duration(0.0);

  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  goal_pose_[0] = initial_pose_[12] ;
  goal_pose_[1] = initial_pose_[13] ;
  goal_pose_[2] = initial_pose_[14] ;
}

void CartesianPoseController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  elapsed_time_ += period;
  double angle, delta_x, delta_y, delta_z, radius;


  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()/ 5.0));

  delta_x = (goal_pose_[0]-current_pose_[12]) * std::sin(angle);
  delta_y = (goal_pose_[1]-current_pose_[13]) * std::sin(angle);
  delta_z = (goal_pose_[2]-current_pose_[14]) * std::sin(angle);

  std::array<double, 16> new_pose = current_pose_;

  new_pose[12] += delta_x;
  new_pose[13] += delta_y;
  new_pose[14] += delta_z;

  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianPoseController::stopping(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  cartesian_pose_handle_->setCommand(initial_pose_);
}

}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::CartesianPoseController, controller_interface::ControllerBase)
