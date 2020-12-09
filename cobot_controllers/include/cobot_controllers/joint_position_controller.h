// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>


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
  void newTargetCallback(const std_msgs::Float64MultiArray::ConstPtr& pose_msg) ;

 private:

  ros::Duration elapsed_time_;

  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  std::array<bool, 7> is_target_reached_{};
  bool target_reached_;

  ros::NodeHandle n_;
  ros::Subscriber sub_cmd_ ;
  ros::Publisher target_reached_pub ;

  std::array<double, 7> goal_pose_{};
  std::array<double, 7> initial_pose_{};
  std::array<double, 7> current_pose_{};

  std::array<double, 7> delta_{};
  std::array<double, 7> err_{};

};

}  // namespace franka_example_controllers
