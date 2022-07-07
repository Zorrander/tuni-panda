// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cobot_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  n_ = ros::NodeHandle(node_handle);

  // JOINT CONTROL 
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
  joint_handles_.resize(7);

  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }
  
  current_joint_values.reserve(7);
  goal_joint_values.reserve(7);

  // IMPEDANCE 

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  /*
  for (size_t i = 0; i < 7; ++i) {
    try {
      //joint_handles_[i] = effort_joint_interface->getHandle(joint_names[i-7]);
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }*/

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  sub_joint_cmd_ = n_.subscribe<sensor_msgs::JointState>("/new_joint_target", 1, &JointPositionController::newJointTargetCallback, this);
  sub_stiffness_cmd_ = n_.subscribe<std_msgs::Float32MultiArray>("/new_stiffness_target", 1, &JointPositionController::newStiffnessCallback, this);
  sub_equilibrium_pose_ = n_.subscribe<geometry_msgs::PoseStamped>("/equilibrium_pose", 1, &JointPositionController::equilibriumPoseCallback, this);
  

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  

  // JOINT POSITION 
  for (size_t i = 0; i < 7; ++i) {
    current_joint_values[i] = joint_handles_[i].getPosition();
    goal_joint_values[i] = joint_handles_[i].getPosition();
    error_[i] = 0.0;
    error_decay_[i] = 0.0;
  }

  // IMPEDANCE
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

  ////////////////////
  const double translational_stiffness{0.0};
  const double rotational_stiffness{0.0};
  ROS_INFO_STREAM("translational_stiffness = " << translational_stiffness);
  ROS_INFO_STREAM("rotational_stiffness = " << rotational_stiffness);
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)     << translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)     << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();  

  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  ////////////////////

  mode = 0;

}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {



  if (mode == 0) {
    // JOINT POSITION

    double k_p, k_d;
    k_p = 1.0;
    k_d = 0.75;
    for (size_t i = 0; i < 7; ++i)
    {
      current_joint_values[i] = joint_handles_[i].getPosition();
      if (error_[4] != 0.0){
        error_decay_[i] = (goal_joint_values[i] - current_joint_values[i]) - error_[i];
      }           
      error_[i] = goal_joint_values[i] - current_joint_values[i] ;
      
      double joint_i_command = k_p * error_[i] + k_d * error_decay_[i] ;
      joint_handles_[i].setCommand(current_joint_values[i] + error_[i]*0.005);
      // ROS_INFO_STREAM("current[" << i << "] = " << current_joint_values[i]);
      // ROS_INFO_STREAM("joint_i_command " << joint_i_command );
      // ROS_INFO_STREAM("command[" << i << "] = " << current_joint_values[i] + joint_i_command*0.005);
    }
  } else {
    // IMPEDANCE
      // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);

    // Desired torque
    tau_d << tau_task + tau_nullspace + coriolis;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 7; i < 14; ++i) {
      joint_handles_[i].setCommand(tau_d(i-7));
    }

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    //ROS_INFO_STREAM("Damping: \n" << cartesian_damping_ << "\n");

    nullspace_stiffness_ =  filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  }

}


Eigen::Matrix<double, 7, 1> JointPositionController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


void JointPositionController::newJointTargetCallback(const sensor_msgs::JointState::ConstPtr& pose_msg)
{
  mode = 0;
  for (size_t i = 0; i < 7; ++i) {
      goal_joint_values[i] = pose_msg->position[i];
      // ROS_INFO_STREAM("command[" << i << "] = " << goal_joint_values[i]);
  }
}

void JointPositionController::newStiffnessCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{ 
  const double translational_stiffness{array->data[0]};
  const double rotational_stiffness{array->data[1]};
  ROS_INFO_STREAM("translational_stiffness = " << translational_stiffness);
  ROS_INFO_STREAM("rotational_stiffness = " << rotational_stiffness);
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)     << translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)     << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();  

  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
}

void JointPositionController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {

  mode = 1;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace cobot_controllers

PLUGINLIB_EXPORT_CLASS(cobot_controllers::JointPositionController,
                       controller_interface::ControllerBase)