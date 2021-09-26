#ifndef CURIOSITY_MARS_ROVER_CONTROLLERS_MAST_CONTROLLER_H
#define CURIOSITY_MARS_ROVER_CONTROLLERS_MAST_CONTROLLER_H

// C++ standard
#include <mutex>

// ROS control
#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

// ROS messages
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_msgs/Angle2D.h>
#include <controller_msgs/SetCommand.h>
#include <controller_msgs/SetMode.h>

// Joint controller
#include <robot_data/joint.h>

// Utils
#include <robot_utils/robot_utils.h>

namespace curiosity_mars_rover_controllers
{

class MastController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  MastController() = default;
  ~MastController() = default;
  
  bool init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);
  
  std::string getEffortInterfaceType() const;
  
  
private:
  /// Joint list
  robot_data::Joint joints_[3];
  int joint_num_ = {3};
  
  /// Joints
  robot_data::Joint& mast_p_joint_ = {joints_[0]};
  robot_data::Joint& mast_02_joint_ = {joints_[1]};
  robot_data::Joint& mast_cameras_joint_ = {joints_[2]};
  
  /// Desired state list
  double desired_states_[3];
  std::mutex desired_state_lock_;
  
  /// Desired states for each joint
  double& mast_p_joint_desired_state_ = {desired_states_[0]};
  double& mast_02_joint_desired_state_ = {desired_states_[1]};
  double& mast_cameras_joint_desired_state_ = {desired_states_[2]};
  
  /// Publishers and subscribers
  ros::Publisher pub_state_;
  ros::Subscriber sub_angle_;
  
  // Services
  ros::ServiceServer srv_command_;
  ros::ServiceServer srv_mode_;
  
  /// State publish time
  ros::Time last_state_published_time_;
  ros::Duration state_publish_period_;
  
  /// Messages
  control_msgs::JointTrajectoryControllerState state_;
  
  void setMastMode(const std::string& mode);
  void publishState(const ros::Time& time);
  
  void setAngleCallback(const controller_msgs::Angle2D::ConstPtr& msg);

  bool setCommandCallback(controller_msgs::SetCommand::Request& req,
                          controller_msgs::SetCommand::Response& resp);
  bool setModeCallback(controller_msgs::SetMode::Request& req,
                       controller_msgs::SetMode::Response& resp);
};

PLUGINLIB_EXPORT_CLASS(curiosity_mars_rover_controllers::MastController, controller_interface::ControllerBase);

}

#endif // CURIOSITY_MARS_ROVER_CONTROLLERS_MAST_CONTROLLER_H
