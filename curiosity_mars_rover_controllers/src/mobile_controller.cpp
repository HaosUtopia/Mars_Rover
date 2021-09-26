#include <curiosity_mars_rover_controllers/mobile_controller.h>

namespace curiosity_mars_rover_controllers
{

  bool MobileController::init(hardware_interface::EffortJointInterface *eff_joint_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    // Init joints
    front_wheel_L_joint_.init("front_wheel_L_joint", "velocity", eff_joint_hw);
    front_wheel_R_joint_.init("front_wheel_R_joint", "velocity", eff_joint_hw);
    middle_wheel_L_joint_.init("middle_wheel_L_joint", "velocity", eff_joint_hw);
    middle_wheel_R_joint_.init("middle_wheel_R_joint", "velocity", eff_joint_hw);
    back_wheel_L_joint_.init("back_wheel_L_joint", "velocity", eff_joint_hw);
    back_wheel_R_joint_.init("back_wheel_R_joint", "velocity", eff_joint_hw);
    suspension_arm_F_L_joint_.init("suspension_arm_F_L_joint", "position", eff_joint_hw);
    suspension_arm_F_R_joint_.init("suspension_arm_F_R_joint", "position", eff_joint_hw);
    suspension_arm_B_L_joint_.init("suspension_arm_B_L_joint", "position", eff_joint_hw);
    suspension_arm_B_R_joint_.init("suspension_arm_B_R_joint", "position", eff_joint_hw);
    suspension_arm_B2_L_joint_.init("suspension_arm_B2_L_joint", "position", eff_joint_hw);
    suspension_arm_B2_R_joint_.init("suspension_arm_B2_R_joint", "position", eff_joint_hw);
    suspension_steer_F_L_joint_.init("suspension_steer_F_L_joint", "position", eff_joint_hw);
    suspension_steer_F_R_joint_.init("suspension_steer_F_R_joint", "position", eff_joint_hw);
    suspension_steer_B_L_joint_.init("suspension_steer_B_L_joint", "position", eff_joint_hw);
    suspension_steer_B_R_joint_.init("suspension_steer_B_R_joint", "position", eff_joint_hw);

    // Set PID parameters for all joints
    front_wheel_L_joint_.setControlParam(10.0, 0.0, 0.0);
    front_wheel_R_joint_.setControlParam(10.0, 0.0, 0.0);
    middle_wheel_L_joint_.setControlParam(10.0, 0.0, 0.0);
    middle_wheel_R_joint_.setControlParam(10.0, 0.0, 0.0);
    back_wheel_L_joint_.setControlParam(10.0, 0.0, 0.0);
    back_wheel_R_joint_.setControlParam(10.0, 0.0, 0.0);
    suspension_arm_F_L_joint_.setControlParam(2200.0, 10.0, 10.0);
    suspension_arm_F_R_joint_.setControlParam(2200.0, 10.0, 10.0);
    suspension_arm_B_L_joint_.setControlParam(4200.0, 10.0, 10.0);
    suspension_arm_B_R_joint_.setControlParam(4200.0, 10.0, 10.0);
    suspension_arm_B2_L_joint_.setControlParam(2200.0, 10.0, 10.0);
    suspension_arm_B2_R_joint_.setControlParam(2200.0, 10.0, 10.0);
    suspension_steer_F_L_joint_.setControlParam(1000.0, 10.0, 100.0);
    suspension_steer_F_R_joint_.setControlParam(1000.0, 10.0, 100.0);
    suspension_steer_B_L_joint_.setControlParam(1000.0, 10.0, 100.0);
    suspension_steer_B_R_joint_.setControlParam(1000.0, 10.0, 100.0);

    // Get structure parameters
    if (!controller_nh.getParam("x_of", x_of_))
    {
      ROS_ERROR_STREAM("Could not find 'x_of' parameter (namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }

    if (!controller_nh.getParam("x_or", x_or_))
    {
      ROS_ERROR_STREAM("Could not find 'x_or' parameter (namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }

    if (!controller_nh.getParam("y_of", y_of_))
    {
      ROS_ERROR_STREAM("Could not find 'y_of' parameter (namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }

    if (!controller_nh.getParam("y_om", y_om_))
    {
      ROS_ERROR_STREAM("Could not find 'y_om' parameter (namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }

    if (!controller_nh.getParam("y_or", y_or_))
    {
      ROS_ERROR_STREAM("Could not find 'y_or' parameter (namespace: " << controller_nh.getNamespace() << ")");
      return false;
    }

    // Calculate motion parameters
    theta_f_ = atan2(x_of_, y_of_);
    theta_r_ = atan2(x_or_, y_or_);
    radius_f_ = sqrt(pow(x_of_, 2.0) + pow(y_of_, 2.0));
    radius_m_ = y_om_;
    radius_r_ = sqrt(pow(x_or_, 2.0) + pow(y_or_, 2.0));

    min_radius_ = std::min(x_of_, x_or_) / std::tan(suspension_steer_F_L_joint_.maxPosition()) + y_om_;
    max_radius_ = std::min(x_of_, x_or_) / std::tan(0.01) + y_om_;

    ROS_INFO_STREAM("theta_f: " << theta_f_);
    ROS_INFO_STREAM("theta_r: " << theta_r_); 
    ROS_INFO_STREAM("min_radius: " << min_radius_);
    ROS_INFO_STREAM("max_radius: " << max_radius_);

    // Get command topic
    std::string command_topic = "/cmd_vel";
    if (!controller_nh.getParam("command_topic", command_topic))
    {
      ROS_WARN_STREAM("Could not find 'command_topic' parameter (namespace: " << controller_nh.getNamespace() << "), \"" << command_topic << "\" will be set as default");
    }

    // Get state publish rate
    double state_publish_rate = 50.0;
    if (!controller_nh.getParam("state_publish_rate", state_publish_rate))
    {
      ROS_WARN_STREAM("Could not find 'state_publish_rate' parameter(namespace: " << controller_nh.getNamespace() << "), " << state_publish_rate << " will be set as default");
    }
    state_publish_period_ = ros::Duration(1.0 / state_publish_rate);

    // Initialize state message
    state_.joint_names = {"radius", "speed"};
    state_.desired.velocities.resize(state_.joint_names.size());
    state_.actual.velocities.resize(state_.joint_names.size());
    state_.error.velocities.resize(state_.joint_names.size());

    // Start the publishers and subscribers
    pub_state_ = controller_nh.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
    sub_command_ = root_nh.subscribe(command_topic, 1, &MobileController::setCommandCallback, this);

    return true;
  }

  void MobileController::starting(const ros::Time &time)
  {
    last_state_published_time_ = time;

    for (int i = 0; i < joint_num_; ++i)
    {
      // joints_[i].updateState();
      joints_[i].resetError();
      desired_states_[i] = joints_[i].currentState();
    }

    setSuspensionMode("ground");
  }

  void MobileController::update(const ros::Time &time, const ros::Duration &period)
  {
    if ((time.nsec % period.nsec) == 0)
    {
      for (int i = 0; i < joint_num_; ++i)
      {
        joints_[i].updateState();
        joints_[i].updateCommand(desired_states_[i], period);
      }

      for (auto &joint : joints_)
      {
        joint.setCommand();
      }

      publishState(time);
      // ROS_INFO_STREAM("joint position: " << suspension_arm_B2_R_joint_.position());
    }
  }

  void MobileController::stopping(const ros::Time &time)
  {
  }

  void MobileController::brake()
  {
    setMotion(0.0, 0.0);
  }

  void MobileController::setSuspensionMode(const std::string &mode)
  {
    desired_state_lock_.lock();

    if (mode == "standard")
    {
      suspension_arm_B2_L_joint_desired_state_ = -0.2;
      suspension_arm_B2_R_joint_desired_state_ = -0.2;
      suspension_arm_B_L_joint_desired_state_ = -0.2;
      suspension_arm_B_R_joint_desired_state_ = -0.2;
      suspension_arm_F_L_joint_desired_state_ = 0.2;
      suspension_arm_F_R_joint_desired_state_ = 0.2;
    }
    else if (mode == "ground")
    {
      suspension_arm_B2_L_joint_desired_state_ = -0.05;
      suspension_arm_B2_R_joint_desired_state_ = -0.05;
      suspension_arm_B_L_joint_desired_state_ = -0.00;
      suspension_arm_B_R_joint_desired_state_ = -0.00;
      suspension_arm_F_L_joint_desired_state_ = -0.09;
      suspension_arm_F_R_joint_desired_state_ = -0.09;
    }

    desired_state_lock_.unlock();
  }

  void MobileController::setMotion(const double &linear, const double &angular)
  {
    desired_state_lock_.lock();

    double radius = std::abs(linear / angular);

    ROS_INFO_STREAM("radius: " << radius);
    ROS_INFO_STREAM("radius > max_radius: " << (radius > max_radius_));
    ROS_INFO_STREAM("radius < -max_radius: " << (radius < -max_radius_));

    if (isnan(radius))
    {
      front_wheel_L_joint_desired_state_ = 0.0;
      front_wheel_R_joint_desired_state_ = 0.0;
      middle_wheel_L_joint_desired_state_ = 0.0;
      middle_wheel_R_joint_desired_state_ = 0.0;
      back_wheel_L_joint_desired_state_ = 0.0;
      back_wheel_R_joint_desired_state_ = 0.0;
    }
    else if (radius < min_radius_) // Turn in mode
    {
      suspension_steer_B_L_joint_desired_state_ = theta_r_;
      suspension_steer_B_R_joint_desired_state_ = -theta_r_;
      suspension_steer_F_L_joint_desired_state_ = -theta_f_;
      suspension_steer_F_R_joint_desired_state_ = theta_f_;

      front_wheel_L_joint_desired_state_ = radius_f_ * -angular;
      front_wheel_R_joint_desired_state_ = radius_f_ * -angular;
      middle_wheel_L_joint_desired_state_ = radius_m_ * -angular;
      middle_wheel_R_joint_desired_state_ = radius_m_ * -angular;
      back_wheel_L_joint_desired_state_ = radius_r_ * -angular;
      back_wheel_R_joint_desired_state_ = radius_r_ * -angular;
    }
    else if (radius > max_radius_) // Stright line mode
    {
      suspension_steer_B_L_joint_desired_state_ = 0.0;
      suspension_steer_B_R_joint_desired_state_ = 0.0;
      suspension_steer_F_L_joint_desired_state_ = 0.0;
      suspension_steer_F_R_joint_desired_state_ = 0.0;

      front_wheel_L_joint_desired_state_ = linear;
      front_wheel_R_joint_desired_state_ = -linear;
      middle_wheel_L_joint_desired_state_ = linear;
      middle_wheel_R_joint_desired_state_ = -linear;
      back_wheel_L_joint_desired_state_ = linear;
      back_wheel_R_joint_desired_state_ = -linear;
    }
    else
    {
      double steer_F_L_angle = std::atan2(x_of_, radius - robot_utils::sgn(angular) * y_om_);
      double steer_F_R_angle = std::atan2(x_of_, radius + robot_utils::sgn(angular) * y_om_);
      double steer_B_L_angle = std::atan2(x_or_, radius - robot_utils::sgn(angular) * y_om_);
      double steer_B_R_angle = std::atan2(x_or_, radius + robot_utils::sgn(angular) * y_om_);

      suspension_steer_F_L_joint_desired_state_ = robot_utils::sgn(angular) * steer_F_L_angle;
      suspension_steer_F_R_joint_desired_state_ = robot_utils::sgn(angular) * steer_F_R_angle;
      suspension_steer_B_L_joint_desired_state_ = -robot_utils::sgn(angular) * steer_B_L_angle;
      suspension_steer_B_R_joint_desired_state_ = -robot_utils::sgn(angular) * steer_B_R_angle;

      front_wheel_L_joint_desired_state_ = x_of_ / std::sin(steer_F_L_angle) * std::abs(angular);
      front_wheel_R_joint_desired_state_ = -x_of_ / std::sin(steer_F_R_angle) * std::abs(angular);
      middle_wheel_L_joint_desired_state_ = robot_utils::sgn(linear) * (radius - robot_utils::sgn(angular) * y_om_) * std::abs(angular);
      middle_wheel_R_joint_desired_state_ = -robot_utils::sgn(linear) * (radius + robot_utils::sgn(angular) * y_om_) * std::abs(angular);
      back_wheel_L_joint_desired_state_ = x_or_ / std::sin(steer_B_L_angle) * std::abs(angular);
      back_wheel_R_joint_desired_state_ = -x_or_ / std::sin(steer_B_R_angle) * std::abs(angular);
    }

    desired_state_lock_.unlock();
  }

  void MobileController::publishState(const ros::Time &time)
  {
    if (last_state_published_time_ + state_publish_period_ < time)
    {
      last_state_published_time_ += state_publish_period_;

      state_.header.stamp = time;

      pub_state_.publish(state_);
    }
  }

  void MobileController::setCommandCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    setMotion(msg->linear.x, msg->angular.z);
  }

} // namespace curiosity_mars_rover_controllers
