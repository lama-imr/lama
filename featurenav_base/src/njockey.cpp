#include <featurenav_base/njockey.h>

namespace featurenav_base {

const ros::Duration NJockey::max_odom_age_ = ros::Duration(0.5);
const double NJockey::reach_angular_distance_ = 0.001;  // (rad)
const int NJockey::histogram_bin_size_ = 20;

NJockey::NJockey(const std::string& name, const std::string& segment_interface_name, const std::string& segment_getter_name) :
  lama_jockeys::NavigatingJockey(name),
  it_(private_nh_),
  forward_velocity_(0.050),
  matcher_max_relative_distance_(0.8),
  max_angular_velocity_(0.5),
  min_angular_velocity_(0.05),
  segment_interface_name_(segment_interface_name),
  segment_getter_name_(segment_getter_name),
  has_odom_(false),
  start_angle_reached_(false),
  segment_end_reached_(false),
  image_processing_running_(false)
{
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  private_nh_.getParamCached("forward_velocity", forward_velocity_);
  private_nh_.getParamCached("kp", kp_);
  private_nh_.getParamCached("matcher_max_relative_distance", matcher_max_relative_distance_);
  private_nh_.getParamCached("max_angular_velocity", max_angular_velocity_);

  twist_publisher_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ROS_DEBUG_STREAM("Waiting for service \"" << segment_getter_name_ << "\"");
  segment_getter_proxy_ = nh_.serviceClient< ::featurenav_base::GetSegment>(segment_getter_name_);
  segment_getter_proxy_.waitForExistence();
}

void NJockey::reset()
{
  has_odom_ = false;
  start_angle_reached_ = false;
  segment_end_reached_ = false;
  image_processing_running_ = false;
}

bool NJockey::retrieveSegment()
{
  lama_interfaces::ActOnMap map_action;
  map_action.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
  map_action.request.object.id = goal_.edge.id;
  map_action.request.interface_name = segment_interface_name_;
  map_agent_.call(map_action);
  if (map_action.response.descriptor_links.empty())
  {
    ROS_DEBUG("No segment associated with vertex %d", map_action.request.object.id);
    return false;
  }
  if (map_action.response.descriptor_links.size() > 1)
  {
    ROS_WARN("More than segment associated with edge %d, taking the first one",
        map_action.request.object.id);
  }
  ::featurenav_base::GetSegment get_segment_srv;
  get_segment_srv.request.id = map_action.response.descriptor_links[0].descriptor_id;
  if (!segment_getter_proxy_.call(get_segment_srv))
  {
    ROS_ERROR_STREAM("Failed to get segment with id " << get_segment_srv.request.id <<
        " and interface " << segment_interface_name_ << " (service " << 
        segment_getter_proxy_.getService() << ")");
    return false;
  }
  segment_ = get_segment_srv.response.descriptor;
  ROS_DEBUG("Received segment %d with %zu landmarks", get_segment_srv.request.id, segment_.landmarks.size());
  return true;
}

void NJockey::onTraverse()
{
  ROS_DEBUG("Received action TRAVERSE");

  if ((canDo == NULL) || (startDo == NULL))
  {
    ROS_ERROR("Internal initialization problem, will not start learning...");
    server_.setAborted();
    return;
  }
  if (extract_features_ == NULL)
  {
    ROS_ERROR("extract_feature function not set, will not start learning...");
    server_.setAborted();
    return;
  }
  if (match_descriptors_ == NULL)
  {
    ROS_ERROR("match_descriptor function not set, will not start learning...");
    server_.setAborted();
    return;
  }

  odom_handler_ = private_nh_.subscribe("odom", 1, &NJockey::callback_odom, this);
  
  // Waiting for the first odometry message.
  ros::Duration(0.2).sleep();
  ros::spinOnce();
  while (!has_odom_ && !server_.isPreemptRequested() && ros::ok())
  {
    ros::spinOnce();
    ROS_WARN_STREAM_THROTTLE(5, "Waiting for odometry on topic " << odom_handler_.getTopic());
    ros::Duration(0.01).sleep();
  }

  ROS_DEBUG("Getting the segment"); // DEBUG

  // Get the segment.
  if (!retrieveSegment())
  {
    ROS_ERROR("No edge with id %d or no segment associated with it", goal_.edge.id);
    server_.setAborted();
    return;
  }

  ROS_DEBUG("Orienting the robot");

  // Orient the robot according to the segment angle.
  ros::Rate r_start_angle(100);
  while(!start_angle_reached_ && !server_.isPreemptRequested() && ros::ok())
  {
    ros::spinOnce();
    geometry_msgs::Twist twist = turnToAngle(segment_.yaw);
    twist_publisher_.publish(twist);
    r_start_angle.sleep();
  }
  
  // Save the position after orienting as starting position.
  start_pose_ = odom_.pose.pose;

  image_handler_ = it_.subscribe("camera/image_raw", 1, &NJockey::callback_image, this);

  ROS_DEBUG("Starting going forward");

  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    if (server_.isPreemptRequested())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      // TODO: should the server be preempted?
      // server_.setPreempted();
      break;
    }

    ROS_DEBUG_THROTTLE(10, "DEBUG 1"); // DEBUG

    if (segment_end_reached_)
    {
      image_handler_.shutdown();
      odom_handler_.shutdown();
      twist_publisher_.publish(geometry_msgs::Twist());
      result_.final_state = result_.DONE;
      result_.completion_time = getCompletionDuration();
      server_.setSucceeded(result_);
      break;
    }
    segment_end_reached_ = false;
    r.sleep();
  }
  ROS_DEBUG("Exiting onTraverse");
}

void NJockey::onStop()
{
  reset();
  image_handler_.shutdown();
  odom_handler_.shutdown();
  twist_publisher_.publish(geometry_msgs::Twist());
  result_.final_state = result_.DONE;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
}

void NJockey::onInterrupt()
{
  image_handler_.shutdown();
  odom_handler_.shutdown();
}

void NJockey::onContinue()
{
  image_handler_ = it_.subscribe("camera/image_raw", 1, &NJockey::callback_image, this);
  odom_handler_ = private_nh_.subscribe("odom", 1, &NJockey::callback_odom, this);
}

void NJockey::callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
  if ((ros::Time::now() - msg->header.stamp) < max_odom_age_)
  {
    odom_ = *msg;
    has_odom_ = true;
  }
}

void NJockey::callback_image(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Time start_time = ros::Time::now();

  if (!has_odom_)
  {
    ROS_WARN("No Odometry received, ignoring image");
    return;
  }

  if ((odom_.header.stamp < msg->header.stamp) && (msg->header.stamp - odom_.header.stamp) > max_odom_age_)
  {
    ROS_WARN("Odometry is too old, ignoring image");
    return;
  }

  geometry_msgs::Twist twist;
  twist.linear.x = forward_velocity_;
  double dtheta;
  processImage(msg, dtheta);

  twist.angular.z = saturate(kp_ * dtheta);
  twist_publisher_.publish(twist);

  ROS_DEBUG("Computation time: %.3f", (ros::Time::now().toSec() - start_time.toSec()));
}

/* GoToGoal behavior for pure rotation
 *
 * direction[in] direction the robot should have at the end (in odometry_ frame).
 * twist[out] set velocity.
 */
geometry_msgs::Twist NJockey::turnToAngle(const double direction)
{
  geometry_msgs::Twist twist;
  if (start_angle_reached_)
  {
    return twist;
  }

  const double yaw_now = tf::getYaw(odom_.pose.pose.orientation);
  const double dtheta = angles::shortest_angular_distance(yaw_now, direction);
  ROS_DEBUG("dtheta to goal: %f", dtheta);

  twist.angular.z = saturate(kp_ * dtheta);
  start_angle_reached_ = (std::abs(dtheta) < reach_angular_distance_);
  return twist;
}

double NJockey::saturate(const double w) const
{
  double w_out = w;

  if (w_out > max_angular_velocity_)
  {
    w_out = max_angular_velocity_;
  }
  else if (w_out < -max_angular_velocity_)
  {
    w_out = -max_angular_velocity_;
  }

  // Dead-zone management.
  if ((w_out > 0) && (w_out < min_angular_velocity_))
  {
    w_out = min_angular_velocity_;
  }
  else if ((w_out < 0) && (w_out > -min_angular_velocity_))
  {
    w_out = -min_angular_velocity_;
  }
  return w_out;
}

/* Return the number of matched landmarks and the angular deviation to learned path
 */
size_t NJockey::processImage(const sensor_msgs::ImageConstPtr& image, double& w)
{
  vector<KeyPoint> keypoints;
  vector<Feature> descriptors;
  extract_features_(image, keypoints, descriptors);
  ROS_DEBUG("Number of detected features: %zu", keypoints.size());

  const double d = distance_from_start();
  vector<Landmark> landmarks = select_tracked_landmarks(d);
  ROS_DEBUG("Number of tracked landmarks: %zu", landmarks.size());

  vector<double> hist = compute_horizontal_differences(landmarks, keypoints, descriptors, d);

  if (hist.empty())
  {
    return 0;
  }
  
  double dtheta = 0;
  double min_value = *std::min_element(hist.begin(), hist.end());
  double max_value = *std::max_element(hist.begin(), hist.end());
  int num_bins = (max_value - min_value) / histogram_bin_size_;
  accumulator_type accumulator(density::num_bins = num_bins, density::cache_size = hist.size());
  accumulator = std::for_each(hist.begin(), hist.end(), accumulator);
  histogram_type histogram = boost::accumulators::density(accumulator);

  // TODO: more histograms (shifted by the pixel) and find maximum in multi-array histogram
  
  int hist_max_i = 0;
  double max_density = -1;
  for (size_t i = 0; i < histogram.size(); ++i)
  {
    if (histogram[i].second > max_density)
    {
      hist_max_i = i;
      max_density = histogram[i].second;
    }
  }
  dtheta = histogram[hist_max_i].first + histogram_bin_size_ / 2.0;

  ROS_DEBUG("Ratio of features close to modus: %.2f %%", histogram[hist_max_i].second * 100.0);
  ROS_DEBUG("Modus of position differences: %.3f pixels", dtheta);

  return dtheta;
}

vector<Landmark> NJockey::select_tracked_landmarks(const double d) const
{
  vector<Landmark> landmarks;
  for (size_t i = 0; i < segment_.landmarks.size(); ++i)
  {
    if ((segment_.landmarks[i].du <= d) && (segment_.landmarks[i].dv >= d))
    {
      landmarks.push_back(segment_.landmarks[i]);
    }
  }
  return landmarks;
}

vector<double> NJockey::compute_horizontal_differences(const vector<Landmark>& landmarks,
    const vector<cv::KeyPoint>& keypoints, const vector<Feature>& descriptors, const double d)
{
  vector<double> hist;

  vector<vector<cv::DMatch> > matches;

  // TODO: compute all matches at once.
  for (size_t i = 0; i < landmarks.size(); ++i)
  {
    matches.clear();
    vector<Feature> query_descriptors(1, landmarks[i].descriptor);
    match_descriptors_(query_descriptors, descriptors, matches);
    if (matches.size() < 1)
    {
      /* ROS_DEBUG("No match found"); */
      continue;
    }
    if (matches[0].size() < 2)
    {
      ROS_ERROR("Not enough matches, found %zu", matches[0].size());
      continue;
    }
    // Add to the horizontal differences.
    if (matches[0][0].distance < matcher_max_relative_distance_ * matches[0][1].distance)
    {
      double dx = (landmarks[i].v.x - landmarks[i].u.x) * (d - landmarks[i].du) / (landmarks[i].dv -
          landmarks[i].dv) +
        landmarks[i].u.x - keypoints[matches[0][0].trainIdx].pt.x;
      if (!isnan(dx))
      {
        hist.push_back(dx);
      }
    }
  }
  return hist;
}

} // namespace featurenav_base


