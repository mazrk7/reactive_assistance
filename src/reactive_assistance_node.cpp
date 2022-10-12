#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <reactive_assistance/obstacle_avoidance.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reactive_assistance_node");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  ROS_INFO_STREAM("Initialiasing the reactive_assistance node");
  reactive_assistance::ObstacleAvoidance obs_avoid(buffer);
  ros::spin();

  return 0;
}