#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goto_crossing");
  ros::NodeHandle n("~");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::spin();
}
