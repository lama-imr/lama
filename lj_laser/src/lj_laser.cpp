#include <lj_laser/lj_laser.h>

namespace lama {
namespace lj_laser {

LJLaser::LJLaser(std::string name, const double frontier_width, const double max_frontier_angle) :
  lama::LocalizingJockey(name),
  scan_received_(false),
  crossing_detector_(frontier_width, max_frontier_angle)
{
}

/* Start the LaserScan subscriber, wait for a LaserScan and exit upon reception.
 */
void LJLaser::getLaserScan()
{
  scan_received_ = false;
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 50, &LJLaser::handleLaser, this);
  ROS_DEBUG("Laser handler started");
  ros::Rate r(100);
  while (ros::ok())
  {
    if (scan_received_)
    {
      // Stop the LaserScan subscriber.
      laserHandler_.shutdown();
      scan_received_ = false;
      break;
    }
    r.sleep();
  }
}

/* Receive a LaserScan message and store it.
 */
void LJLaser::handleLaser(const sensor_msgs::LaserScan msg)
{
  ROS_DEBUG("NJLaser: laser arrived with %zu beams", msg.ranges.size());

  scan_ = msg;
  scan_received_ = true;
}

/* Save the vertex descriptors associated with the current robot position in the database.
 *
 * The descriptor are a LaserScan, a list of double (x, y, r), and a list of frontier angles.
 */
void LJLaser::onGetVertexDescriptor()
{
  getLaserScan();
  ros::Time start_time = ros::Time::now();

  // Add the LaserScan to the database.
  lama_interfaces::SetVectorLaserScan scan_ds;
  scan_ds.request.descriptor.push_back(scan_);
  ros::service::call("laser_descriptor_setter", scan_ds);
  result_.descriptors.push_back(scan_ds.response.id);

  // Add the list of double (x, y, r).
  double x;
  double y;
  double r;
  crossing_detector_.crossingCenter(scan_, x, y, r);
  lama_interfaces::SetVectorDouble vdouble_ds;
  vdouble_ds.request.descriptor.push_back(x);
  vdouble_ds.request.descriptor.push_back(y);
  vdouble_ds.request.descriptor.push_back(r);
  ros::service::call("vector_double_setter", vdouble_ds);
  result_.descriptors.push_back(vdouble_ds.response.id);

  // Add the list of frontier angles.
  std::vector<Frontier> frontiers;
  crossing_detector_.frontiers(scan_, frontiers);
  lama_interfaces::SetVectorDouble ds;
  for (std::vector<Frontier>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it)
  {
    ds.request.descriptor.push_back(it->angle);
  }
  ros::service::call("vector_double_setter", ds);
  result_.descriptors.push_back(ds.response.id);
  
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Time::now() - start_time;
  server_.setSucceeded(result_);
}

void LJLaser::onGetEdgesDescriptors()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LJLaser::onLocalizeInVertex()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LJLaser::onLocalizeEdge()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void LJLaser::onGetSimilarity()
{
  getLaserScan();
  ros::Time start_time = ros::Time::now();

  // Transform the scan into a polygon (move scanToPolygon to lama_common).
  // Get all scans from database.
  // Compare them to the current scan by calling one of the pm_??? service.
  // Return what?
  
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Time::now() - start_time;
  server_.setSucceeded(result_);
}

} // namespace lj_laser
} // namespace lama
