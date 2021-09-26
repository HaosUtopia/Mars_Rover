#include <curiosity_mars_rover_navigation/odom_broadcaster.h>

namespace curiosity_mars_rover_navigation
{

  OdomBroadcaster::OdomBroadcaster(ros::NodeHandle &nh)
  {
    nh_ = nh;

    // Get odom topic name
    std::string odom_topic = "odom";
    if (!nh_.getParam("odom_topic", odom_topic))
    {
      ROS_WARN_STREAM("Could not find 'odom_topic' parameter (namespace: " << nh_.getNamespace() << "), \"" << odom_topic << "\" will be set as default");
    }

    // Get tf publish rate
    double tf_publish_rate = 5.0;
    if (!nh_.getParam("tf_publish_rate", tf_publish_rate))
    {
      ROS_WARN_STREAM("Could not find 'tf_publish_rate' parameter(namespace: " << nh_.getNamespace() << "), " << tf_publish_rate << " will be set as default");
    }
    tf_publish_period_ = ros::Duration(1.0 / tf_publish_rate);
    last_tf_published_time_ = ros::Time(0.0);

    sub_odom_ = nh_.subscribe(odom_topic, 5, &OdomBroadcaster::odomCallback, this);
  }

  void OdomBroadcaster::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (msg->header.stamp - last_tf_published_time_ > tf_publish_period_)
    {
      last_tf_published_time_ = msg->header.stamp;
      
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header = msg->header;
      odom_trans.child_frame_id = msg->child_frame_id;
      odom_trans.transform.translation.x = msg->pose.pose.position.x;
      odom_trans.transform.translation.y = msg->pose.pose.position.y;
      odom_trans.transform.translation.z = msg->pose.pose.position.z;
      odom_trans.transform.rotation = msg->pose.pose.orientation;

      odom_broadcaster_.sendTransform(odom_trans);
    }
  }

} // namespace curiosity_mars_rover_navigation