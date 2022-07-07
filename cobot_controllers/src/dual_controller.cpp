// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cobot_controllers/joint_position_controller.h>



namespace cobot_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  n_ = ros::NodeHandle(node_handle);
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

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

   std::vector<double> cartesian_stiffness_vector;
   std::vector<double> cartesian_damping_vector;
   desired_force_torque = Eigen::VectorXd(6);
   for (int i=0;i<6;i++)
      desired_force_torque[i] =0;

   external_torque = node_handle.advertise<geometry_msgs::TwistStamped>("measured_torque",1);
   
   

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

     
     for (size_t i = 0; i < 7; ++i) {
       try {
         joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
       } catch (const hardware_interface::HardwareInterfaceException& ex) {
         ROS_ERROR_STREAM(
             "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
         return false;
       }
     }

   dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

   dynamic_server_compliance_param_ = std::make_unique<  dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
         dynamic_reconfigure_compliance_param_node_);
   dynamic_server_compliance_param_->setCallback( boost::bind(&JointPositionController::complianceParamCallback, this, _1, _2));

   position_d_.setZero();
   orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
   position_d_target_.setZero();
   orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

   cartesian_stiffness_.setZero();
   cartesian_damping_.setZero();


  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {

      sub_joint_cmd_ = n_.subscribe<sensor_msgs::JointState>("/new_joint_target", 1, &JointPositionController::newJointTargetCallback, this);
      sub_stiffness_cmd_ =       n_.subscribe<std_msgs::Float32MultiArray>("/new_impedance_target", 1, &JointPositionController::newStiffnessCallback, this);
      sub_equilibrium_pose_ = n_.subscribe<geometry_msgs::PoseStamped>("/equilibrium_pose", 1, &JointPositionController::equilibriumPoseCallback, this);

      current_joint_values.reserve(7);
      goal_joint_values.reserve(7);

      for (size_t i = 0; i < 7; ++i) {
        current_joint_values[i] = joint_handles_[i].getPosition();
        goal_joint_values[i] = joint_handles_[i].getPosition();
        error_[i] = 0.0;
        error_decay_[i] = 0.0;
      }
      
      // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
      // to initial configuration
      franka::RobotState initial_state = state_handle_->getRobotState();
      // get jacobian
      std::array<double, 42> jacobian_array =  model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
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
      mode = 0;
}



void JointPositionController::newJointTargetCallback(const sensor_msgs::JointState::ConstPtr& pose_msg)
{
  mode = 0;
  for (size_t i = 0; i < 7; ++i) {
      goal_joint_values[i] = pose_msg->position[i];
  }

}


void JointPositionController::newStiffnessCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{ 
  const double translational_stiffness{array->data[0]};
  const double rotational_stiffness{array->data[1]};
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << translational_stiffness * Eigen::Matrix3d::Identity();
      //<< config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << rotational_stiffness * Eigen::Matrix3d::Identity();
      //<< config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
      //<< 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();  
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {



    if (mode == 0) {

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
            joint_handles_[i].setCommand(current_joint_values[i] + joint_i_command*0.005);
            // ROS_INFO_STREAM("command[" << i << "] = " << current_joint_values[i] + joint_i_command);
      }
      ROS_INFO_STREAM(" error");

    } else {

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
        //Eigen::Matrix<double, 3, 1> ori_error1;
        //Eigen::Quaterniond error_quaternion_v1(orientation.inverse() * orientation_d_);
        //ori_error1.head(3) << error_quaternion_v1.x(), error_quaternion_v1.y(), error_quaternion_v1.z();
        
        /*
        ROS_INFO_STREAM(" error: \n" << error(0) << "\n" << error(1) << "\n"<< error(2) << "\n");
        */

        // Transform to base frame
        //ori_error1.head(3) << -transform.linear() * ori_error1.head(3);
        
       
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
        // convert to axis angle
        //Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
        // compute "orientation error"
        // error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) <<  -transform.linear() * error.tail(3);
        //ROS_INFO_STREAM(" orientaton err v1: " << ori_error1 << "\n");
        //ROS_INFO_STREAM(" orientaton err v2: " << error.tail(3) << "\n");

        //compute control
        // allocate variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

        // pseudoinverse for nullspace handling
        // kinematic pseuoinverse
        Eigen::MatrixXd jacobian_transpose_pinv;
        // pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv, false);
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        

        // Cartesian PD control with damping ratio = 1
        tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
        // nullspace PD control with damping ratio = 1
        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                             jacobian.transpose() * jacobian_transpose_pinv) *
                                (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                                   (2.0 * sqrt(nullspace_stiffness_)) * dq);

        /*
        for (size_t i = 0; i < 7; ++i) {
            ROS_INFO_STREAM(" delta qs: " << tau_task(i) << ", " << tau_nullspace(i) << ", "<< coriolis(i) << ", " << "\n");
        }*/


        // Desired torque
        tau_d << tau_task + tau_nullspace + coriolis;
        // tau_d << tau_task  + coriolis;

        // Saturate torque rate to avoid discontinuities
        tau_d << saturateTorqueRate(tau_d, tau_J_d);


        
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i)); 
            //ROS_INFO_STREAM(" delta q: " << tau_d(i) << "\n");
        }
        /*for (size_t i = 0; i < 7; ++i)
        {
              current_joint_values[i] = joint_handles_[i].getPosition();
              joint_handles_[i].setCommand(current_joint_values[i]);
        }*/

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
                           const Eigen::Matrix<double, 7, 1>& tau_J_d) 
{  // NOLINT (readability-identifier-naming)
   Eigen::Matrix<double, 7, 1> tau_d_saturated{};
   for (size_t i = 0; i < 7; i++) 
   {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), - delta_tau_max_);
   }
   
   return tau_d_saturated;
}

void JointPositionController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  const double translational_stiffness{200.00};
  const double rotational_stiffness{60.00};
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << translational_stiffness * Eigen::Matrix3d::Identity();
      //<< config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << rotational_stiffness * Eigen::Matrix3d::Identity();
      //<< config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
      //<< 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();
      //<< 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}


void JointPositionController::equilibriumPoseCallback( const geometry_msgs::PoseStampedConstPtr& msg) 
{
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