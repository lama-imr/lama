#include <nj_laser/jockey.h>

namespace lama {
namespace nj_laser {

Jockey::Jockey(std::string name) :
  lama::NavigatingJockey(name)
{
	pub_crossing_marker_ = nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  pub_exits_marker_ = nh_.advertise<visualization_msgs::Marker> ("exits_marker", 50, true);
  pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void Jockey::onTraverse()
{
  unsetGoalReached();
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  ROS_DEBUG("Laser handler started");
  
  ros::Rate r(50);
  while (ros::ok() && goal_.action == lama_jockeys::NavigateGoal::TRAVERSE)
  {
    std::vector<double> exitAngles = cross_detector.getExitAngles();
    ROS_DEBUG("Crossing detected with %zu exits", exitAngles.size());
    if (exitAngles.size() > 2)
    {
      geometry_msgs::Point goal;
      goal.x = cross_detector.getCrossCenterX();
      goal.y = cross_detector.getCrossCenterY();
      geometry_msgs::Twist twist = goToGoal(goal);
      pub_twist_.publish(twist);
      if (isGoalReached())
      {
        ROS_DEBUG("Jockey: goal reached");
        result_.final_state = lama_jockeys::NavigateResult::DONE;
        result_.completion_time = ros::Time::now() - getStartTime() - getInterruptionsDuration();
        server_.setSucceeded(result_);
        laserHandler_.shutdown();
        break;
      }
    }
    else
    {
      // Go straight if no crossing is detected.
      geometry_msgs::Point goal;
      goal.x = 0.5;
      geometry_msgs::Twist twist = goToGoal(goal);
      pub_twist_.publish(twist);
    }
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("Exiting onTraverse");
}

void Jockey::onStop()
{
  laserHandler_.shutdown();
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onInterrupt()
{
  onStop();
}

void Jockey::onContinue()
{
  onTraverse();
}

void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  ROS_DEBUG("Jockey: laser arrived with %zu beams", msg->ranges.size());

  cross_detector.crossDetect(*msg);
  std::vector<double> desc = cross_detector.getCrossDescriptor();
  ROS_DEBUG("Crossing x: %.3f, y: %.3f, r: %.3f, nroads: %zu", desc[0], desc[1], desc[2], desc.size() - 3);

  // Visualization: a sphere at detected crossing center
	if (pub_crossing_marker_.getNumSubscribers())
	{
		visualization_msgs::Marker m = crossingMarker(msg->header.frame_id,
				cross_detector.getCrossCenterX(), cross_detector.getCrossCenterY(), cross_detector.getCrossRadius());
		pub_crossing_marker_.publish(m);
	}


  // Visualization: a line at each detected road
	if (pub_exits_marker_.getNumSubscribers())
	{
		std::vector<double> exitAngles = cross_detector.getExitAngles();
		visualization_msgs::Marker m = exitsMarker(msg->header.frame_id, exitAngles, cross_detector.getCrossRadius());
		pub_exits_marker_.publish(m);
	}
}

} // namespace nj_laser
} // namespace lama

