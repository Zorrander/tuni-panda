#include <cobot_controllers/cartesian_pose_controller.h>
#include <unistd.h>
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
    ROS_ERROR("CartesianPoseController: C#include <Matrix3x3.h>ould not get parameter arm_id");
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
  ROS_INFO_STREAM("pose_msg->position.y = " << pose_msg->position.y);
  ROS_INFO_STREAM("pose_msg->position.z = " << pose_msg->position.z);

  ROS_INFO_STREAM("pose_msg->orientation.x = " << pose_msg->orientation.x);
  ROS_INFO_STREAM("pose_msg->orientation.y = " << pose_msg->orientation.y);
  ROS_INFO_STREAM("pose_msg->orientation.z = " << pose_msg->orientation.z);
  ROS_INFO_STREAM("pose_msg->orientation.w = " << pose_msg->orientation.w);

  goal_pose_[0] = pose_msg->position.x ;
  goal_pose_[1] = pose_msg->position.y ;
  goal_pose_[2] = pose_msg->position.z ;

  goal_pose_[3] = pose_msg->orientation.x ;
  goal_pose_[4] = pose_msg->orientation.y ;
  goal_pose_[5] = pose_msg->orientation.z ;
  goal_pose_[6] = pose_msg->orientation.w ;

  Eigen::Quaterniond q = Eigen::Quaterniond(goal_pose_[6], goal_pose_[3], goal_pose_[4], goal_pose_[5]);
  rot_mat_d = q.normalized().toRotationMatrix();
  ROS_INFO_STREAM("R= " << rot_mat_d);

  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseController::starting(const ros::Time& /* time */) {
  ROS_INFO("CartesianPoseController starting");
  elapsed_time_ = ros::Duration(0.0);

  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  goal_pose_[0] = initial_pose_[12] ;
  goal_pose_[1] = initial_pose_[13] ;
  goal_pose_[2] = initial_pose_[14] ;

  goal_pose_[3] = 0.0 ;
  goal_pose_[4] = 0.0 ;
  goal_pose_[5] = 0.0 ;
  goal_pose_[6] = 1.0 ;

  Eigen::Quaterniond q = Eigen::Quaterniond(goal_pose_[6], goal_pose_[3], goal_pose_[4], goal_pose_[5]);
  rot_mat_d = q.normalized().toRotationMatrix();
  ROS_INFO_STREAM("R= " << rot_mat_d);
}

void CartesianPoseController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  elapsed_time_ += period;
  double angle, delta_x, delta_y, delta_z, radius, err_x, err_y, err_z;

  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  std::array<double, 16> new_pose = current_pose_;

  //for (size_t i = 0; i < 16; ++i) {
  //  ROS_INFO_STREAM("current_pose_[" << i << " = " << current_pose_[i]);
  //}

  pose_err_[0] = rot_mat_d(0)-current_pose_[0];
  pose_err_[1] = rot_mat_d(1)-current_pose_[1];
  pose_err_[2] = rot_mat_d(2)-current_pose_[2];

  pose_err_[4] = rot_mat_d(4)-current_pose_[4];
  pose_err_[5] = rot_mat_d(5)-current_pose_[5];
  pose_err_[6] = rot_mat_d(6)-current_pose_[6];

  pose_err_[8] =  rot_mat_d(8)-current_pose_[8];
  pose_err_[9] =  rot_mat_d(9)-current_pose_[9];
  pose_err_[10] = rot_mat_d(10)-current_pose_[10];


  pose_err_[12] = goal_pose_[0]-current_pose_[12];
  pose_err_[13] = goal_pose_[1]-current_pose_[13];
  pose_err_[14] = goal_pose_[2]-current_pose_[14];

  if (pose_err_[0] > 0.01 || pose_err_[1] > 0.01 || pose_err_[2] > 0.01) {
    //ROS_INFO_STREAM("err_x = " << err_x);
    //ROS_INFO_STREAM("err_y = " << err_y);
    //ROS_INFO_STREAM("err_z = " << err_z);

    //ROS_INFO_STREAM("DISTANCE = " << 20.0*sqrt(pow(err_x, 2.0)+pow(err_y, 2.0)+pow(err_z, 2.0)));

    // angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()/ (3.0 + 20.0*sqrt(pow(err_x, 2.0)+pow(err_y, 2.0)+pow(err_z, 2.0)))));
    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()/ 5.0 ));

    new_pose[0] = pose_err_[0]* std::sin(angle);
    new_pose[1] = pose_err_[1]* std::sin(angle);
    new_pose[2] = pose_err_[2]* std::sin(angle);

    new_pose[4] = pose_err_[4]* std::sin(angle);
    new_pose[5] = pose_err_[5]* std::sin(angle);
    new_pose[6] = pose_err_[6]* std::sin(angle);

    new_pose[8] =  pose_err_[8]* std::sin(angle);
    new_pose[9] =  pose_err_[9]* std::sin(angle);
    new_pose[10] = pose_err_[10]* std::sin(angle);

    new_pose[12] += pose_err_[12] * std::sin(angle);
    new_pose[13] += pose_err_[13] * std::sin(angle);
    new_pose[14] += pose_err_[14] * std::sin(angle);
  }

  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianPoseController::stopping(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  cartesian_pose_handle_->setCommand(initial_pose_);
}

}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::CartesianPoseController, controller_interface::ControllerBase)
