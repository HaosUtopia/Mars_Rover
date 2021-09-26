#ifndef CURIOSITY_MARS_ROVER_CONTROLLERS_ARM_CONTROLLER_H
#define CURIOSITY_MARS_ROVER_CONTROLLERS_ARM_CONTROLLER_H

// C++ standard
#include <mutex>

// ROS control
#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

// ROS messages
#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_msgs/SetCommand.h>
#include <controller_msgs/SetMode.h>

// Joint controller
#include <robot_data/joint.h>

namespace curiosity_mars_rover_controllers
{

class ArmController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ArmController() = default;
  ~ArmController() = default;
  
  bool init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);
  
  std::string getEffortInterfaceType() const;
  
private:
  /// Joint list
  robot_data::Joint joints_[5];
  int joint_num_ = {5};
  
  /// Joints
  robot_data::Joint& arm_01_joint_ = {joints_[0]};
  robot_data::Joint& arm_02_joint_ = {joints_[1]};
  robot_data::Joint& arm_03_joint_ = {joints_[2]};
  robot_data::Joint& arm_04_joint_ = {joints_[3]};
  robot_data::Joint& arm_tools_joint_ = {joints_[4]};
  
  /// Desired state list
  double desired_states_[5];
  std::mutex desired_state_lock_;
  
  /// Desired states for each joint
  double& arm_01_joint_desired_state_ = {desired_states_[0]};
  double& arm_02_joint_desired_state_ = {desired_states_[1]};
  double& arm_03_joint_desired_state_ = {desired_states_[2]};
  double& arm_04_joint_desired_state_ = {desired_states_[3]};
  double& arm_tools_joint_desired_state_ = {desired_states_[4]};
  
  /// Publishers and subscribers
  ros::Publisher pub_state_;
  
  /// Services
  ros::ServiceServer srv_command_;
  ros::ServiceServer srv_mode_;
  
  /// State publish time
  ros::Time last_state_published_time_;
  ros::Duration state_publish_period_;
  
  /// Messages
  control_msgs::JointTrajectoryControllerState state_;
  
  void setArmMode(const std::string& mode);
  void publishState(const ros::Time& time);
  
  bool setCommandCallback(controller_msgs::SetCommand::Request& req,
                          controller_msgs::SetCommand::Response& resp);
  bool setModeCallback(controller_msgs::SetMode::Request& req,
                       controller_msgs::SetMode::Response& resp);
};

PLUGINLIB_EXPORT_CLASS(curiosity_mars_rover_controllers::ArmController, controller_interface::ControllerBase);

}

#endif // CURIOSITY_MARS_ROVER_CONTROLLERS_ARM_CONTROLLER_H
