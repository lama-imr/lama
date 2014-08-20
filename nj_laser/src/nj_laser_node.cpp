/**
 * Large Map 
 * Laser based memoryless (reactive) navigating jockey
 *
 */

#include <string>

#include <ros/ros.h>

#include <nj_laser/nj_laser.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_jockey");
  ros::NodeHandle n;
  
  std::string navigating_jockey_name;
  std::string default_jockey_name = ros::this_node::getName();
  default_jockey_name += "_jockey";
	n.param<std::string>("navigating_jockey_name", navigating_jockey_name, default_jockey_name);

  NJLaser jockey(navigating_jockey_name);

  std::string info = ros::this_node::getName();
  info += " and ";
  info += navigating_jockey_name;
  info += " started.";
  ROS_INFO("%s", info.c_str());
  ros::spin();

}

#undef intro
