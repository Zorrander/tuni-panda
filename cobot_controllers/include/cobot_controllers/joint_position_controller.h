// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace cobot_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;

  double k_p;
  double k_d;
  double T_d;

  std::array<double, 7> error_{};
  std::array<double, 7> error_decay_{};

  std::vector<double> current_command_{};
  std::vector<double> current_joint_values;
  std::vector<double> goal_joint_values;

  bool found_ik;
  std::array<bool, 7> is_target_reached_{};
  bool target_reached_;

  robot_state::RobotStatePtr kinematic_state;
  robot_model::RobotModelPtr kinematic_model ;
  robot_state::JointModelGroup* joint_model_group ;

  ros::NodeHandle n_;
  ros::Subscriber sub_cmd_ ;
  ros::Publisher target_reached_pub ;

  void newTargetCallback(const geometry_msgs::Pose::ConstPtr& pose_msg) ;

};

}  // namespace franka_example_controllers
