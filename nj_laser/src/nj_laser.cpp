#include <nj_laser/nj_laser.h>

namespace lama {
namespace nj_laser {

NJLaser::NJLaser(std::string name) : lama::interfaces::NavigatingJockey(name)
{
	pub_crossing_marker_ = nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  pub_exits_marker_ = nh_.advertise<visualization_msgs::Marker> ("exits_marker", 50, true);
  pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void NJLaser::onTraverse()
{
  unsetGoalReached();
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &NJLaser::handleLaser, this);
  ROS_DEBUG("Laser handler started");
  
  ros::Rate r(50);
  while (ros::ok() && goal_.action == lama_interfaces::NavigateGoal::TRAVERSE)
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
        result_.final_state = lama_interfaces::NavigateResult::DONE;
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
    r.sleep();
  }
}

void NJLaser::onStop()
{
  laserHandler_.shutdown();
  result_.final_state = lama_interfaces::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void NJLaser::onInterrupt()
{
  onStop();
}

void NJLaser::onContinue()
{
  onTraverse();
}

void NJLaser::handleLaser(const sensor_msgs::LaserScan msg)
{
  ROS_DEBUG("NJLaser: laser arrived with %zu beams", msg.ranges.size());

  cross_detector.crossDetect(msg);
  std::vector<double> desc = cross_detector.getCrossDescriptor();
  ROS_DEBUG("Crossing x: %.3f, y: %.3f, r: %.3f, nroads: %zu", desc[0], desc[1], desc[2], desc.size() - 3);

  // Visualization: a sphere at detected crossing center
	if (pub_crossing_marker_.getNumSubscribers())
	{
		visualization_msgs::Marker m = crossingMarker(msg.header.frame_id,
				cross_detector.getCrossCenterX(), cross_detector.getCrossCenterY(), cross_detector.getCrossRadius());
		pub_crossing_marker_.publish(m);
	}


  // Visualization: a line at each detected road
	if (pub_exits_marker_.getNumSubscribers())
	{
		std::vector<double> exitAngles = cross_detector.getExitAngles();
		visualization_msgs::Marker m = exitsMarker(msg.header.frame_id, exitAngles, cross_detector.getCrossRadius());
		pub_exits_marker_.publish(m);
	}
}

} // namespace nj_laser
} // namespace lama

