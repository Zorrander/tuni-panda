#include <cobot_controllers/joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot.h>


namespace cobot_controllers {

  bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                            ros::NodeHandle& node_handle) {
    n_ = ros::NodeHandle(node_handle);
    velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
    // n = new NodeHandle(node_handle);
    if (velocity_joint_interface_ == nullptr) {
      ROS_ERROR(
          "JointVelocityController: Error getting velocity joint interface from hardware!");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
      ROS_ERROR("JointVelocityController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
      ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                       << joint_names.size() << " instead of 7 names!");
      return false;
    }
    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
      try {
        velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "JointVelocityController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR("JointVelocityController: Could not get state interface from hardware");
      return false;
    }

    sub_cmd_ = node_handle.subscribe<std_msgs::Float32MultiArray>("jointVelocities", 10, &JointVelocityController::commandCB, this);

    return true;
  }

  void JointVelocityController::commandCB(const std_msgs::Float32MultiArray::ConstPtr& array)
  {
      int i = 0;
      for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
      {
              Arr[i] = *it;
              i++;
      }
  }


  void JointVelocityController::starting(const ros::Time& /* time */) {
    elapsed_time_ = ros::Duration(0.0);
  }

  void JointVelocityController::update(const ros::Time& /* time */,
                                              const ros::Duration& period) {

      /*
      franka::Robot robot("130.230.37.115");
      size_t count = 0;
      robot.read([&count](const franka::RobotState& robot_state) {
        // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        std::cout << robot_state << "\n" << std::endl;

        std::cout << "---------------------------------------------\n\n\n" << std::endl;
        return count++ < 100;
      });
      std::cout << "Done." << std::endl;
      */

      int index = 0;
      for (auto joint_handle : velocity_joint_handles_) {
          //std::cout << "/* VELOCITIES */" << Arr[index] << '\n';
          joint_handle.setCommand(Arr[index]);
          index++;
      }
  }

  void JointVelocityController::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  }



  }  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointVelocityController, controller_interface::ControllerBase)
