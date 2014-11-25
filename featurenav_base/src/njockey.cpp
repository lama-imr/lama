#include <featurenav_base/njockey.h>

namespace lama {
namespace featurenav_base {

NJockey::NJockey(const std::string& name) :
  NavigatingJockey(name),
  it_(private_nh_),
  segment_end_reached_(false)
{
}

void NJockey::onTraverse()
{
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": Received action TRAVERSE or CONTINUE");

  image_handler_ = it_.subscribe("image", 1, &NJockey::callback_image, this);
  
  ros::Rate r(100);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    if (segment_end_reached_)
    {
      image_handler_.shutdown();
      result_.final_state = result_.DONE;
      result_.completion_time = getCompletionDuration();
      server_.setSucceeded(result_);
      break;
    }
    segment_end_reached_ = false;
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("Exiting onTraverse");
}

void NJockey::onStop()
{
}

void NJockey::onInterrupt()
{
}

void NJockey::onContinue()
{
}

void NJockey::callback_image(const sensor_msgs::ImageConstPtr& msg)
{
}

} // namespace featurenav_base
} // namespace lama


