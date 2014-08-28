/**
 * Localizing jockey based on LaserScan associated with
 * absolute heading.
 *
 */

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <lj_laser_heading/jockey.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizing_jockey");
  ros::NodeHandle n("~");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string localizing_jockey_server;
  std::string default_server_name = ros::this_node::getName();
  default_server_name += "_server";
	n.param<std::string>("localizing_jockey_server_name", localizing_jockey_server, default_server_name);

  double frontier_width;
  n.param<double>("frontier_width", frontier_width, 0.3);

  double max_frontier_dist;
  n.param<double>("max_frontier_distance", max_frontier_dist, 3.0);

  lama::lj_laser_heading::Jockey jockey(localizing_jockey_server, frontier_width, max_frontier_dist);

  ROS_INFO("%s started (with action server %s)", ros::this_node::getName().c_str(), localizing_jockey_server.c_str());
  ros::spin();
}

