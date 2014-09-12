#include <lj_laser_heading/jockey.h>

namespace lama {
namespace lj_laser_heading {

// Maximum time interval between reception of LaserScan and Pose, whichever
// comes first. The jockey will reject all data until one LaserScan message and
// one Pose message come shortly one after another.
// TODO: write as a parameter.
const ros::Duration Jockey::max_data_time_delta_ = ros::Duration(0.050);

Jockey::Jockey(std::string name, const double frontier_width, const double max_frontier_angle) :
  lama::LocalizingJockey(name),
  data_received_(false),
  scan_reception_time_(ros::Time(0)),
  pose_reception_time_(ros::Time(0)),
  laser_interface_name_(name + "_laser_descriptor"),
  crossing_interface_name_(name + "_crossing_descriptor"),
  similarity_server_name_("similarity_server"),
  crossing_detector_(frontier_width, max_frontier_angle)
{
  // Initialize the client for the similarity server.
  similarity_server_ = nh_.serviceClient<polygon_matcher::PolygonSimilarity>(similarity_server_name_);
}

/* Create the getter and setter services for LaserScan descriptors.
 */
void Jockey::initMapLaserInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = laser_interface_name_;
  srv.request.get_service_message = "lama_interfaces/GetVectorLaserScan";
  srv.request.set_service_message = "lama_interfaces/SetVectorLaserScan";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service AddInterface");
  }
  if (!srv.response.success)
  {
    ROS_ERROR("Failed to create the Lama interface");
  }
  // Initialize the clients for the LaserScan getter and setter services (interface to map).
  laser_descriptor_getter_ = nh_.serviceClient<lama_interfaces::GetVectorLaserScan>(srv.response.get_service_name);
  laser_descriptor_getter_.waitForExistence();
  laser_descriptor_setter_ = nh_.serviceClient<lama_interfaces::SetVectorLaserScan>(srv.response.set_service_name);
  laser_descriptor_setter_.waitForExistence();
}

/* Create the setter services for Crossing descriptors.
 */
void Jockey::initMapCrossingInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = laser_interface_name_;
  srv.request.get_service_message = "lama_msgs/GetCrossing";
  srv.request.set_service_message = "lama_msgs/SetCrossing";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service AddInterface");
  }
  if (!srv.response.success)
  {
    ROS_ERROR("Failed to create the Lama interface");
  }
  // Initialize the client for the Crossing setter services (interface to map).
  crossing_descriptor_setter_ = nh_.serviceClient<lama_msgs::SetCrossing>(srv.response.set_service_name);
  crossing_descriptor_setter_.waitForExistence();
}

/* Start the subscribers, wait for a LaserScan and a Pose, and exit upon reception.
 */
void Jockey::getData()
{
  data_received_ = false;
  laserHandler_ = nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  poseHandler_ = nh_.subscribe<geometry_msgs::Pose>("pose", 1, &Jockey::handlePose, this);
  ROS_DEBUG("LaserScan and Pose handler started");
  laserHandler_.getTopic();
  ros::Rate r(100);
  while (ros::ok())
  {
    if (data_received_)
    {
      // Stop the subscribers (may be superfluous).
      laserHandler_.shutdown();
      poseHandler_.shutdown();
      data_received_ = false;
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
}

/* Receive a LaserScan message and store it.
 */
void Jockey::handleLaser(const sensor_msgs::LaserScan msg)
{
  ROS_DEBUG("lj_laser_heading::Jockey: laser arrived with %zu beams", msg.ranges.size());

  scan_ = msg;
  scan_reception_time_ = msg.header.stamp;
  data_received_ = (scan_reception_time_ - pose_reception_time_) < max_data_time_delta_;
}

/* Receive a Pose message and store it.
 */
void Jockey::handlePose(const geometry_msgs::Pose msg)
{
  ROS_DEBUG("lj_laser_heading::Jockey: pose arrived");

  pose_ = msg;
  pose_reception_time_ = ros::Time::now();
  data_received_ = (pose_reception_time_ - scan_reception_time_) < max_data_time_delta_;
}

/* Return the vertex descriptors associated with the current robot position through result_.
 *
 * The descriptor are a LaserScan and a Crossing.
 */
// TODO: Discuss with Karel the exact role of onGetVertexDescriptor
// TODO: in particular: should it save something in the database? This jockey
// is not a learning jockey.
void Jockey::onGetVertexDescriptor()
{
  getData();

  // Rotate scan_ so that scan_.angle_min and scan_.angle_max are absolute angles.
  rotateScan();

  // Add the LaserScan to the descriptor list.
  lama_interfaces::SetVectorLaserScan vscan_setter;
  vscan_setter.request.descriptor.push_back(scan_);
  laser_descriptor_setter_.call(vscan_setter);
  result_.descriptors.push_back(vscan_setter.response.id);

  // Add the Crossing to the descriptor list.
  lama_msgs::SetCrossing crossing_setter;
  lama_msgs::Crossing crossing;
  double x;
  double y;
  double r;
  crossing_detector_.crossingCenter(scan_, x, y, r);
  crossing.center.x = x;
  crossing.center.y = y;
  crossing.radius = r;
  std::vector<lama_msgs::Frontier> frontiers;
  crossing_detector_.frontiers(scan_, frontiers);
  for (std::vector<lama_msgs::Frontier>::const_iterator it = frontiers.begin(); it != frontiers.end(); ++it)
  {
    crossing.frontiers.push_back(*it);
  }
  crossing_descriptor_setter_.call(crossing_setter);
  result_.descriptors.push_back(crossing_setter.response.id);
  
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Time::now() - getStartTime() - getInterruptionsDuration();
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
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeInVertex()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeEdge()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
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
  lama_interfaces::ActOnMap srv;
  srv.request.action.action = lama_interfaces::MapAction::GET_VERTEX_LIST;
  map_agent_.call(srv);
  
  // Iterate over vertices and get the associated Polygon (from the LaserScan).
  std::vector<int32_t> vertex_indexes;
  vertex_indexes.reserve(srv.response.objects.size());
  std::vector<geometry_msgs::Polygon> polygons;
  polygons.reserve(srv.response.objects.size());
  for (size_t i = 0; i < srv.response.objects.size(); ++i)
  {
    // Get all descriptors associated with the current vertex.
    lama_interfaces::ActOnMap desc_srv;
    desc_srv.request.action.action = lama_interfaces::MapAction::PULL_VERTEX;
    desc_srv.request.object.id = srv.response.objects[i].id;
    map_agent_.call(desc_srv);
    // Transform the LaserScan into a Polygon.
    for (size_t j = 0; j < desc_srv.response.descriptors.size(); ++j)
    {
      if (desc_srv.response.descriptors[j].interface_name == laser_interface_name_)
      {
        lama_interfaces::GetVectorLaserScan scan_srv;
        scan_srv.request.id.descriptor_id = desc_srv.response.descriptors[j].descriptor_id;
        laser_descriptor_getter_.call(scan_srv);
        geometry_msgs::Polygon polygon = scanToPolygon(scan_srv.response.descriptor[0]);
        vertex_indexes.push_back(desc_srv.response.descriptors[j].object_id);
        polygons.push_back(polygon);
      }
    }
  }
  
  // Compare them to the current polygon by calling one of the pm_* service.
  polygon_matcher::PolygonSimilarity simi_srv;
  simi_srv.request.polygon1 = current_polygon;
  result_.idata.reserve(vertex_indexes.size());
  result_.fdata.reserve(vertex_indexes.size());
  for (size_t i = 0; i < vertex_indexes.size(); ++i)
  {
    simi_srv.request.polygon2 = polygons[i];
    similarity_server_.call(simi_srv);
    result_.idata.push_back(vertex_indexes[i]);
    result_.fdata.push_back(simi_srv.response.raw_similarity);
  }

  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Time::now() - start_time;
  server_.setSucceeded(result_);
}

} // namespace lj_laser_heading
} // namespace lama
