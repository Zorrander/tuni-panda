#pragma once

#include <string>
#include <vector>
#include <array>
#include "std_msgs/Float32MultiArray.h"
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>


namespace cobot_controllers {

class JointVelocityController
    : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping (const ros::Time& time);

 private:
   hardware_interface::VelocityJointInterface* velocity_joint_interface_;
   std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
   ros::Duration elapsed_time_;
   std::vector<float> joint_velocities_;
   ros::Subscriber sub_cmd_ ;
   void commandCB(const std_msgs::Float32MultiArray::ConstPtr& array) ;
   float Arr[12]; // joint v
   ros::NodeHandle n_;
};

}  // namespace cobot_controllers
