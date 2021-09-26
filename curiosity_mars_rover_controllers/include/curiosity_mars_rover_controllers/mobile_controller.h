#ifndef CURIOSITY_MARS_ROVER_CONTROLLERS_MOBILE_CONTROLLER_H
#define CURIOSITY_MARS_ROVER_CONTROLLERS_MOBILE_CONTROLLER_H

// C++ standard
#include <mutex>
#include <cmath>

// ROS control
#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// Joint controller
#include <robot_data/joint.h>

namespace curiosity_mars_rover_controllers
{

class MobileController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  MobileController() = default;
  ~MobileController() = default;
  
  bool init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);
  
  std::string getEffortInterfaceType() const;
  
private:
  /// Joint list
  robot_data::Joint joints_[16];
  int joint_num_ = {16};
  
  /// Joints
  robot_data::Joint& front_wheel_L_joint_ = {joints_[0]};
  robot_data::Joint& front_wheel_R_joint_ = {joints_[1]};
  robot_data::Joint& middle_wheel_L_joint_ = {joints_[2]};
  robot_data::Joint& middle_wheel_R_joint_ = {joints_[3]};
  robot_data::Joint& back_wheel_L_joint_ = {joints_[4]};
  robot_data::Joint& back_wheel_R_joint_ = {joints_[5]};
  robot_data::Joint& suspension_arm_F_L_joint_ = {joints_[6]};
  robot_data::Joint& suspension_arm_F_R_joint_ = {joints_[7]};
  robot_data::Joint& suspension_arm_B_L_joint_ = {joints_[8]};
  robot_data::Joint& suspension_arm_B_R_joint_ = {joints_[9]};
  robot_data::Joint& suspension_arm_B2_L_joint_ = {joints_[10]};
  robot_data::Joint& suspension_arm_B2_R_joint_ = {joints_[11]};
  robot_data::Joint& suspension_steer_F_L_joint_ = {joints_[12]};
  robot_data::Joint& suspension_steer_F_R_joint_ = {joints_[13]};
  robot_data::Joint& suspension_steer_B_L_joint_ = {joints_[14]};
  robot_data::Joint& suspension_steer_B_R_joint_ = {joints_[15]};
  
  /// Desired state list
  double desired_states_[16];
  std::mutex desired_state_lock_;
  
  /// Desired states for each joint
  double& front_wheel_L_joint_desired_state_ = {desired_states_[0]};
  double& front_wheel_R_joint_desired_state_ = {desired_states_[1]};
  double& middle_wheel_L_joint_desired_state_ = {desired_states_[2]};
  double& middle_wheel_R_joint_desired_state_ = {desired_states_[3]};
  double& back_wheel_L_joint_desired_state_ = {desired_states_[4]};
  double& back_wheel_R_joint_desired_state_ = {desired_states_[5]};
  double& suspension_arm_F_L_joint_desired_state_ = {desired_states_[6]};
  double& suspension_arm_F_R_joint_desired_state_ = {desired_states_[7]};
  double& suspension_arm_B_L_joint_desired_state_ = {desired_states_[8]};
  double& suspension_arm_B_R_joint_desired_state_ = {desired_states_[9]};
  double& suspension_arm_B2_L_joint_desired_state_ = {desired_states_[10]};
  double& suspension_arm_B2_R_joint_desired_state_ = {desired_states_[11]};
  double& suspension_steer_F_L_joint_desired_state_ = {desired_states_[12]};
  double& suspension_steer_F_R_joint_desired_state_ = {desired_states_[13]};
  double& suspension_steer_B_L_joint_desired_state_ = {desired_states_[14]};
  double& suspension_steer_B_R_joint_desired_state_ = {desired_states_[15]};

  /// Structure parameters
  double x_of_;
  double x_or_;
  double y_of_;
  double y_om_;
  double y_or_;

  // Motion parameters
  double min_radius_;
  double max_radius_;
  double theta_f_; // Turn in angle for front wheels
  double theta_r_; // Turn in angle for rear wheels
  double radius_f_;
  double radius_m_;
  double radius_r_;
  
  /// Publishers and subscribers
  ros::Publisher pub_state_;
  ros::Subscriber sub_command_;
  
  /// State publish time
  ros::Time last_state_published_time_;
  ros::Duration state_publish_period_;
  
  /// Messages
  control_msgs::JointTrajectoryControllerState state_;
  
  void brake();
  void setSuspensionMode(const std::string& mode);
  void setMotion(const double& linear, const double& angular);
  void publishState(const ros::Time& time);
  
  void setCommandCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

PLUGINLIB_EXPORT_CLASS(curiosity_mars_rover_controllers::MobileController, controller_interface::ControllerBase);

}


#endif // CURIOSITY_MARS_ROVER_CONTROLLERS_MOBILE_CONTROLLER_H
