#include <lj_laser_heading/jockey.h>

namespace lama {
namespace lj_laser_heading {

const ros::Duration Jockey::max_data_time_delta_ = ros::Duration(0.050);

Jockey::Jockey(std::string name, const double frontier_width, const double max_frontier_angle) :
  lama::interfaces::LocalizingJockey(name),
  data_received_(false),
  scan_reception_time_(ros::Time(0)),
  pose_reception_time_(ros::Time(0)),
  crossing_detector_(frontier_width, max_frontier_angle)
{
}

/* Start the subscribers, wait for a LaserScan and a Pose, and exit upon reception.
 */
void Jockey::getData()
{
  data_received_ = false;
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  poseHandler_ = nh_.subscribe<geometry_msgs::Pose>("pose", 1, &Jockey::handlePose, this);
  ROS_DEBUG("LaserScan and Pose handler started");
  ros::Rate r(100);
  while (ros::ok())
  {
    if (data_received_)
    {
      // Stop the subscribers.
      laserHandler_.shutdown();
      poseHandler_.shutdown();
      data_received_ = false;
      break;
    }
    r.sleep();
  }
}

/* Receive a LaserScan message and store it.
 */
void Jockey::handleLaser(const sensor_msgs::LaserScan msg)
{
  ROS_DEBUG("lj_laser_heading::Jockey: laser arrived with %zu beams", msg.ranges.size());

  scan_ = msg;
  scan_reception_time_ = ros::Time::now();
  if ((scan_reception_time_ - pose_reception_time_) < max_data_time_delta_)
    data_received_ = true;
}

/* Receive a Pose message and store it.
 */
void Jockey::handlePose(const geometry_msgs::Pose msg)
{
  ROS_DEBUG("lj_laser_heading::Jockey: pose arrived");

  pose_ = msg;
  pose_reception_time_ = ros::Time::now();
  if ((pose_reception_time_ - scan_reception_time_) < max_data_time_delta_)
    data_received_ = true;
}

/* Save the vertex descriptors associated with the current robot position in the database.
 *
 * The descriptor are a LaserScan, a list of double (x, y, r), and a list of frontier angles.
 */
void Jockey::onGetVertexDescriptor()
{
  getData();
  ros::Time start_time = ros::Time::now();

  // Rotate scan_ so that scan_.angle_min and scan_.angle_max are absolute angles.
  rotateScan();

  // Add the LaserScan to the database.
  lama_interfaces::lmi_laser_descriptor_set scan_ds;
  scan_ds.request.descriptor.push_back(scan_);
  ros::service::call("lmi_laser_descriptor_setter", scan_ds);
  result_.descriptors.push_back(scan_ds.response.id);

  // Add the list of double (x, y, r).
  double x;
  double y;
  double r;
  crossing_detector_.crossingCenter(scan_, x, y, r);
  lama_interfaces::lmi_vector_double_set vdouble_ds;
  vdouble_ds.request.descriptor.push_back(x);
  vdouble_ds.request.descriptor.push_back(y);
  vdouble_ds.request.descriptor.push_back(r);
  ros::service::call("lmi_vector_double_descriptor_setter", vdouble_ds);
  result_.descriptors.push_back(vdouble_ds.response.id);

  // Add the list of frontier angles.
  std::vector<Frontier> frontiers;
  crossing_detector_.frontiers(scan_, frontiers);
  lama_interfaces::lmi_vector_double_set ds;
  for (std::vector<Frontier>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it)
  {
    ds.request.descriptor.push_back(it->angle);
  }
  ros::service::call("lmi_vector_double_descriptor_setter", ds);
  result_.descriptors.push_back(ds.response.id);
  
  result_.state = lama_interfaces::LocalizeResult::DONE;
  result_.completion_time = ros::Time::now() - start_time;
  server_.setSucceeded(result_);
}

/* Return the angle from a quaternion representing a rotation around the z-axis
 *
 * The quaternion in ROS is q = (w, x, y, z), so that
 * q = (cos(a/2), ux * sin(a/2), uy * sin(a/2), uz * sin(a/2)),
 *   where a is the rotation angle and (ux, uy, uz) is the unit vector of the
 *   rotation axis.
 * For a rotation around z, we have q = (cos(a/2), 0, 0, sin(a/2)). Thus
 * a = 2 * atan2(z, w).
 */
double angleFromQuaternion(const geometry_msgs::Quaternion& q)
{
  if (std::fabs(q.x) > 1e-5 || std::fabs(q.y) > 1e-5)
  {
    ROS_WARN("Laser frame rotation is not around the z-axis, just pretending it is");
  }
  return 2 * std::atan2(q.z, q.w);
}

/* Change angle_min and angle_max of scan_, so that they represent absolute angles.
 *
 * This as for consequence that angle_min or angle_max can be outside of ]-pi,pi].
 */
void Jockey::rotateScan()
{
  double heading = angleFromQuaternion(pose_.orientation);
  scan_.angle_min -= heading;
  scan_.angle_max -= heading;
}

void Jockey::onGetEdgesDescriptors()
{
  result_.state = lama_interfaces::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeInVertex()
{
  result_.state = lama_interfaces::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeEdge()
{
  result_.state = lama_interfaces::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onGetSimilarity()
{
  getData();
  ros::Time start_time = ros::Time::now();

  // Transform the scan into a polygon.
  geometry_msgs::Polygon current_polygon = scanToPolygon(scan_);

  // Get all scans from database.
  // Compare them to the current scan by calling one of the pm_??? service.
  // Return what?
  
  result_.state = lama_interfaces::LocalizeResult::DONE;
  result_.completion_time = ros::Time::now() - start_time;
  server_.setSucceeded(result_);
}

} // namespace lj_laser_heading
} // namespace lama
