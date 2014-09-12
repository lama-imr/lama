/**
 * Large Map 
 * Laser based memoryless (reactive) navigating jockey
 *
 */

#include <string>

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <nj_laser/jockey.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_jockey");
  ros::NodeHandle n("~");
  
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string navigating_jockey_name;
  std::string default_jockey_name = ros::this_node::getName();
  default_jockey_name += "_server";
	n.param<std::string>("navigating_jockey_server_name", navigating_jockey_name, default_jockey_name);

  lama::nj_laser::Jockey jockey(navigating_jockey_name);

  ROS_INFO("%s started (with server %s)", ros::this_node::getName().c_str(), navigating_jockey_name.c_str());
  ros::spin();
  return 0;
}

