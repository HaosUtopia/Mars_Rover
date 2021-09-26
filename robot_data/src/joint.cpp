#include<robot_data/joint.h>

namespace robot_data
{

bool Joint::init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface)
{
  return init(joint_name, joint_type, joint_interface, ros::NodeHandle());
}

bool Joint::init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, ros::NodeHandle nh)
{
  joint_handle_ = joint_interface->getHandle(joint_name);
  
  if (joint_type == "position")
  {
    joint_type_ = POSITION;
  }
  else if (joint_type == "velocity")
  {
    joint_type_ = VELOCITY;
  }
  else if (joint_type == "effort")
  {
    joint_type_ = EFFORT;
  }
  else
  {
    ROS_ERROR_STREAM("Unrecognized joint type: " << joint_type);
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", nh))
  {
    ROS_ERROR("Failed to parse urdf file from parameter 'robot_description'");
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
  if (!joint_urdf)
  {
    ROS_ERROR_STREAM("Could not find joint " << joint_name << " in urdf");
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  is_continous_ = joint_urdf->type == urdf::Joint::CONTINUOUS;
  min_limit_[0] = joint_urdf->limits->lower;
  max_limit_[0] = joint_urdf->limits->upper;
  min_limit_[1] = -joint_urdf->limits->velocity;
  max_limit_[1] = joint_urdf->limits->velocity;
  min_limit_[2] = -joint_urdf->limits->effort;
  max_limit_[2] = joint_urdf->limits->effort;

  if (is_continous_)
  {
    min_limit_[0] = -std::numeric_limits<double>::infinity();
    max_limit_[0] = std::numeric_limits<double>::infinity();
  }


  return true;
}

void Joint::setControlParam(const double& Kp, const double& Ki, const double Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void Joint::resetError()
{
  ep_ = 0.0;
  ei_ = 0.0;
  ed_ = 0.0;
}

void Joint::updateState()
{
  joint_state_[0] = joint_handle_.getPosition();
  joint_state_[1] = joint_handle_.getVelocity();
  joint_state_[2] = joint_handle_.getEffort();
}

void Joint::updateCommand(const double& desired_state, const ros::Duration& period)
{
  double ep = std::max(std::min(desired_state, max_limit_[joint_type_]), min_limit_[joint_type_]) - joint_state_[joint_type_];
  double ed = (ep - ep_) / period.toSec();
  
  if (period == ros::Duration(0.0) || std::isnan(ep) || std::isinf(ep) || std::isnan(ed) || std::isinf(ed))
  {
    joint_command_ = 0.0;
    return;
  }
  
  ep_ = ep;
  ed_ = ed;
  ei_ += ep_ * period.toSec();
  
  joint_command_ = Kp_ * ep_ + Kd_ * ed_ + Ki_ * ei_;
}

void Joint::setCommand()
{
  joint_handle_.setCommand(joint_command_);
}

const bool& Joint::isContinous() const
{
  return is_continous_;
}

const double& Joint::minPosition() const
{
  return min_limit_[0];
}

const double& Joint::maxPosition() const
{
  return max_limit_[0];
}

const double& Joint::maxVelocity() const
{
  return max_limit_[1];
}

const double& Joint::maxEffort() const
{
  return max_limit_[2];
}

const double& Joint::currentState() const
{
  return joint_state_[joint_type_];
}

const double& Joint::position() const
{
  return joint_state_[0];
}

const double& Joint::velocity() const
{
  return joint_state_[1];
}

const double& Joint::effort() const
{
  return joint_state_[2];
}

}
