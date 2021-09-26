#ifndef ROBOT_DATA_JOINT_H
#define ROBOT_DATA_JOINT_H

#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <robot_utils/robot_utils.h>

namespace robot_data
{

class Joint
{
public:
  Joint() = default;
  ~Joint() = default;
  
  bool init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface);
  bool init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, ros::NodeHandle nh);
  void setControlParam(const double& Kp, const double& Ki, const double Kd);
  void updateState();
  void updateCommand(const double& desired_state, const ros::Duration& period);
  void setCommand();
  void resetError();
  
  const bool& isContinous() const;
  const double& minPosition() const;
  const double& maxPosition() const;
  const double& maxVelocity() const;
  const double& maxEffort() const;

  const double& currentState() const;
  const double& position() const;
  const double& velocity() const;
  const double& effort() const;
private:
  /// Joint parameter
  hardware_interface::JointHandle joint_handle_;
  enum JointType {POSITION = 0, VELOCITY = 1, EFFORT = 2} joint_type_;

  /// Joint limit
  bool is_continous_;
  double min_limit_[3]; // [min_pos, min_vel, min_eff]
  double max_limit_[3]; // [max_pos, max_vel, max_eff]
  
  /// Current joint state
  double joint_state_[3];
  
  /// Next joint command
  double joint_command_;
  
  /// Control parameter
  double Kp_ = {0.0};
  double Kd_ = {0.0};
  double Ki_ = {0.0};
  
  /// Error for control
  double ep_ = {0.0};
  double ed_ = {0.0};
  double ei_ = {0.0};
};

}

#endif // ROBOT_DATA_JOINT_H
