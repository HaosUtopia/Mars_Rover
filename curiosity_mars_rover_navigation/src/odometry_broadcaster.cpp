#include <curiosity_mars_rover_navigation/odom_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_broadcaster");

  ros::NodeHandle nh;
  curiosity_mars_rover_navigation::OdomBroadcaster odom(nh);

  ros::spin();
}