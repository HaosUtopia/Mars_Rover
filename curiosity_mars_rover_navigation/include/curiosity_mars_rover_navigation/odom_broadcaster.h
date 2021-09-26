#ifndef CURIOSITY_MARS_ROVER_NAVIGATION_ODOM_BROADCASTER_H
#define CURIOSITY_MARS_ROVER_NAVIGATION_ODOM_BROADCASTER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace curiosity_mars_rover_navigation
{

class OdomBroadcaster
{
public:
  OdomBroadcaster(ros::NodeHandle& nh);
  ~OdomBroadcaster() = default;

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  tf::TransformBroadcaster odom_broadcaster_;

  ros::Time last_tf_published_time_;
  ros::Duration tf_publish_period_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

}

#endif // CURIOSITY_MARS_ROVER_NAVIGATION_ODOM_BROADCASTER_H