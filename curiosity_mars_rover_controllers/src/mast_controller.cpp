#include <curiosity_mars_rover_controllers/mast_controller.h>

namespace curiosity_mars_rover_controllers
{

bool MastController::init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Init joints
  mast_p_joint_.init("mast_p_joint", "position", eff_joint_hw);
  mast_02_joint_.init("mast_02_joint", "position", eff_joint_hw);
  mast_cameras_joint_.init("mast_cameras_joint", "position", eff_joint_hw);
  
  // Set PID parameters for all joints
  mast_p_joint_.setControlParam(500.0, 10.0, 1000.0);
  mast_02_joint_.setControlParam(500.0, 10.0, 100.0);
  mast_cameras_joint_.setControlParam(100.0, 10.0, 100.0);
  
  // Get service server name
  std::string command_topic = "set_command";
  if (!controller_nh.getParam("command_topic", command_topic))
  {
    ROS_WARN_STREAM("Could not find 'command_topic' parameter (namespace: " << controller_nh.getNamespace() << "), \"" << command_topic << "\" will be set as default");
  }

  std::string angle_topic = "rotate_cam";
  if (!controller_nh.getParam("angle_topic", angle_topic))
  {
    ROS_WARN_STREAM("Could not find 'angle_topic' parameter (namespace: " << controller_nh.getNamespace() << "), \"" << angle_topic << "\" will be set as default");
  }
  
  std::string mode_topic = "set_mode";
  if (!controller_nh.getParam("mode_topic", mode_topic))
  {
    ROS_WARN_STREAM("Could not find 'mode_topic' parameter (namespace: " << controller_nh.getNamespace() << "), \"" << mode_topic << "\" will be set as default");
  }
  
  // Get state publish rate
  double state_publish_rate = 50.0;
  if (!controller_nh.getParam("state_publish_rate", state_publish_rate))
  {
    ROS_WARN_STREAM("Could not find 'state_publish_rate' parameter(namespace: " << controller_nh.getNamespace() << "), " << state_publish_rate << " will be set as default");
  }
  state_publish_period_ = ros::Duration(1.0 / state_publish_rate);
  
  // Initialize state message
  state_.joint_names = {"mast_p_joint", "mast_02_joint", "mast_cameras_joint"};
  state_.desired.positions.resize(state_.joint_names.size());
  state_.actual.positions.resize(state_.joint_names.size());
  state_.error.positions.resize(state_.joint_names.size());
  
  // Start the publishers and subscribers
  pub_state_ = controller_nh.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
  sub_angle_ = controller_nh.subscribe(angle_topic, 1, &MastController::setAngleCallback, this);

  // Start the service server
  srv_command_ = controller_nh.advertiseService(command_topic, &MastController::setCommandCallback, this);
  srv_mode_ = controller_nh.advertiseService(mode_topic, &MastController::setModeCallback, this);
  
  return true;
}

void MastController::starting(const ros::Time& time)
{
  last_state_published_time_ = time;
  
  for (int i = 0; i < joint_num_; ++i)
  {
    // joints_[i].updateState();
    joints_[i].resetError();
    desired_states_[i] = joints_[i].currentState();
  }
  
  setMastMode("close");
}

void MastController::update(const ros::Time& time, const ros::Duration& period)
{
  if ( (time.nsec % period.nsec) == 0)
  {
    for (int i = 0; i < joint_num_; ++i)
    {
      joints_[i].updateState();
      joints_[i].updateCommand(desired_states_[i], period);
    }
    
    for (auto& joint : joints_)
    {
      joint.setCommand();
    }
    
    publishState(time);
  }
}

void MastController::stopping(const ros::Time& time)
{

}

void MastController::setMastMode(const std::string& mode)
{
  desired_state_lock_.lock();
  
  if (mode == "close")
  {
    mast_p_joint_desired_state_ = 1.57;
    mast_02_joint_desired_state_ = -1.57;
    mast_cameras_joint_desired_state_ = 0.0;
  }
  
  if (mode == "open")
  {
    mast_p_joint_desired_state_ = 0.0;
    mast_02_joint_desired_state_ = -0.5;
    mast_cameras_joint_desired_state_ = 0.0;
  }
  
  desired_state_lock_.unlock();
}

void MastController::publishState(const ros::Time& time)
{
  if (last_state_published_time_ + state_publish_period_ < time)
  {
    last_state_published_time_ += state_publish_period_;
    
    state_.header.stamp = time;
    
    for (int i = 0; i < joint_num_; ++i)
    {
      state_.desired.positions[i] = desired_states_[i];
      state_.actual.positions[i] = joints_[i].currentState();
      state_.error.positions[i] = state_.desired.positions[i] - state_.actual.positions[i];
    }
    
    pub_state_.publish(state_);
  }
}

void MastController::setAngleCallback(const controller_msgs::Angle2D::ConstPtr& msg)
{
  desired_state_lock_.lock();

  mast_02_joint_desired_state_ = std::abs(mast_02_joint_.currentState() - msg->x - mast_02_joint_desired_state_) > 0.1 ? mast_02_joint_.currentState() - msg->x : mast_02_joint_desired_state_;
  mast_cameras_joint_desired_state_ = std::abs(mast_cameras_joint_.currentState() - msg->y - mast_cameras_joint_desired_state_) > 0.1 ? robot_utils::normalize_angle(mast_cameras_joint_.currentState() - msg->y) : mast_cameras_joint_desired_state_;
  
  desired_state_lock_.unlock();
}

bool MastController::setCommandCallback(controller_msgs::SetCommand::Request& req,
                                        controller_msgs::SetCommand::Response& resp)
{
  if (req.command.command.values.size() != joint_num_)
  {
    ROS_ERROR_STREAM("The command size " << req.command.command.values.size() << " is not equal to joint number " << joint_num_);
    resp.success = false;
    resp.status_message = "Command size doesn't match";
    return true;
  }
  
  desired_state_lock_.lock();
  
  for (int i = 0; i < joint_num_; ++i)
  {
    // TODO: Add joint range detection
    desired_states_[i] = req.command.command.values[i];
  }
  
  desired_state_lock_.unlock();
  
  resp.success = true;
  return true;
}

bool MastController::setModeCallback(controller_msgs::SetMode::Request& req,
                                     controller_msgs::SetMode::Response& resp)
{
  if (req.mode.mode_name == "open" || req.mode.mode_name == "close")
  {
    setMastMode(req.mode.mode_name);
    resp.success = true;
  }
  else
  {
    resp.success = false;
    resp.status_message = "Mode name should be 'open' or close";
  }
  
  return true;
}

}
