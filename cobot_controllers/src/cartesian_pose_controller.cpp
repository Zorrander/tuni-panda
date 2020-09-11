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

  /*
  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseController: Exception getting state handle: " << e.what());
    return false;
  }
  */

  return true;
}

void CartesianPoseController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);

  k_p = 0.2;  // damping ratio
  k_d = 0.5;  // natural frequency

  goal_pose_[0] = 0.46 ;
  goal_pose_[1] = 0.01 ;
  goal_pose_[2] = 0.20 ;

  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  error_[0] = goal_pose_[0] - current_pose_[12];
  error_[1] = goal_pose_[1] - current_pose_[13];
  error_[2] = goal_pose_[2] - current_pose_[14];
}

void CartesianPoseController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

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

  double multiplier = 0.0001 ;
  x_d = x_d*multiplier ;
  y_d = y_d*multiplier ;
  z_d = z_d*multiplier ;

  ROS_INFO_STREAM("Sending command: " << " x_d=" << x_d << " y_d=" << y_d << " z_d=" << z_d) ;

  std::array<double, 16> new_pose = current_pose_;
  new_pose[12] += x_d;
  new_pose[13] += y_d;
  new_pose[14] += z_d;

  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianPoseController::stopping(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  cartesian_pose_handle_->setCommand(initial_pose_);
}

}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::CartesianPoseController, controller_interface::ControllerBase)
