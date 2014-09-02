/* Base class for jockeys
 *
 */

#include <lama_interfaces/jockey.h>

namespace lama
{
namespace interfaces
{

Jockey::Jockey(std::string name) :
  jockey_name_(name)
{
  map_agent_ = nh_.serviceClient<lama_interfaces::ActOnMap>("lama_map_agent");
  map_agent_.waitForExistence();
}

void Jockey::initAction()
{
  start_time_ = ros::Time::now();
  interrupted_ = false;
  interruption_time_ = ros::Time(0);
  resume_time_ = ros::Time(0);
  interruptions_duration_ = ros::Duration(0);
}

void Jockey::interrupt()
{
  if (!interrupted_)
  {
    interrupted_ = true;
    interruption_time_ = ros::Time::now();
  }
}

void Jockey::resume()
{
  if (interrupted_)
  {
    interrupted_ = false;
    resume_time_ = ros::Time::now();
    interruptions_duration_ += resume_time_ - interruption_time_;
  }
}

void Jockey::onInterrupt()
{
  ROS_DEBUG("%s: action interrupted", jockey_name_.c_str());
}

void Jockey::onContinue()
{
  ROS_DEBUG("%s: action resumed", jockey_name_.c_str());
}

} // namespace interfaces
} // namespace lama

