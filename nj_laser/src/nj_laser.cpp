#include <nj_laser/nj_laser.h>

NJLaser::NJLaser(std::string name) : lama::interfaces::NavigatingJockey(name)
{
	crossing_marker_ = nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  exits_marker_ = nh_.advertise<visualization_msgs::Marker> ("exits_marker", 50, true);
}

void NJLaser::onTraverse()
{
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan> ("base_scan", 50, &NJLaser::handleLaser, this);
  ros::spin();
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

  cl_.crossDetect(msg);
  std::vector<double> desc = cl_.getCrossDescriptor();
  ROS_INFO("CROSS x: %.3f, y: %.3f, r: %.3f, nroads: %lu", desc[0], desc[1], desc[2], desc.size() - 3);

  // Visualization: a sphere at detected crossing center
	if (crossing_marker_.getNumSubscribers())
	{
		visualization_msgs::Marker m = crossingMarker(msg.header.frame_id,
				cl_.getCrossCenterX(), cl_.getCrossCenterY(), cl_.getCrossRadius());
		crossing_marker_.publish(m);
	}


  // Visualization: a line at each detected road
	if (exits_marker_.getNumSubscribers())
	{
		std::vector<double> exitAngles = cl_.getExitAngles();
		visualization_msgs::Marker m = exitsMarker(msg.header.frame_id, exitAngles, cl_.getCrossRadius());
		exits_marker_.publish(m);
	}
}


