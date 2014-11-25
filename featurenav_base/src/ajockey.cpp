#include <featurenav_base/ajockey.h>

namespace lama {
namespace featurenav_base {

const ros::Duration AJockey::max_odom_age_ = ros::Duration(0.5);
const double AJockey::matcher_equality_multiplication_ = 0.8;
const double AJockey::min_landmark_dist_ = 0.02;
const ros::Duration AJockey::max_landmark_age_ = ros::Duration(2.0);

AJockey::AJockey(const std::string& name, const std::string& segment_interface_name, const std::string& segment_setter_name) :
  LearningJockey(name),
  it_(private_nh_),
  segment_interface_name_(segment_interface_name),
  segment_setter_name_(segment_setter_name),
  has_odom_(false),
  learning_started_(false),
  image_processing_running_(false)
{
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG_STREAM(ros::this_node::getName() << ": waiting for service \"" << segment_setter_name_ << "\"");
  segment_setter_proxy_ = nh_.serviceClient< ::featurenav_base::SetSegment>(segment_setter_name_);
  segment_setter_proxy_.waitForExistence();
}

void AJockey::reset()
{
  has_odom_ = false;
  learning_started_ = false;
  segment_.distance = 0;
  segment_.landmarks.clear();
  landmarks_.clear();
  image_processing_running_ = false;
}

void AJockey::onStartLearn()
{
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": received ON_START_LEARN");

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

  image_handler_ = it_.subscribe("camera/image_raw", 1, &AJockey::callback_image, this);
  odom_handler_ = private_nh_.subscribe("odom", 1, &AJockey::callback_odom, this);

  ros::Rate r(100);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      // TODO: should the server be preempted?
      // server_.setPreempted();
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("Exiting onStartLearn");
}

void AJockey::onStopLearn()
{
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": received ON_STOP_LEARN");

  if (server_.isPreemptRequested() && !ros::ok())
  {
    ROS_INFO("%s: Preempted", jockey_name_.c_str());
    // set the action state to preempted
    // TODO: should the server be preempted?
    // server_.setPreempted();
    image_handler_.shutdown();
    odom_handler_.shutdown();
    reset();
    return;
  }

  image_handler_.shutdown();
  odom_handler_.shutdown();

  ROS_DEBUG_STREAM(ros::this_node::getName() << ": waiting for processImage to finish");

  // Wait for processImage to finish.
  ros::Rate r(1000);
  while (ros::ok())
  {
    if (!image_processing_running_)
    {
      break;
    }
    ros::spinOnce();
  }

  ROS_DEBUG_STREAM(ros::this_node::getName() << ": processImage finished");

  segment_.distance = distance_from_start();
  /* for (size_t i = 0; i < landmarks_.size(); ++i) */
  /* { */
  /*   segment_.landmarks.push_back(landmarks_[i].landmark); */
  /* } */

  // Save the segment into the database and return its id.
  ROS_INFO_STREAM(ros::this_node::getName() << ": saving a segment with " <<
      segment_.landmarks.size() << " landmarks");
  ::featurenav_base::SetSegment segment_setter;
  segment_setter.request.descriptor = segment_;
  if (!segment_setter_proxy_.call(segment_setter))
  {
    ROS_ERROR("Failed to add Segment to the map");
    server_.setAborted();
    return;
  }
  ROS_INFO("Added Segment with id %d", segment_setter.response.id); // DEBUG
  result_.descriptor_link = segmentDescriptorLink(segment_setter.response.id);
  server_.setSucceeded(result_);
  reset();
}

void AJockey::onInterrupt()
{
  // TODO: discuss with Karel what to do on interrupt:
  // - wait and watch features, stop segment if moving?
  // - stop the current segment without condition?
}

void AJockey::onContinue()
{
}

void AJockey::callback_image(const sensor_msgs::ImageConstPtr& msg)
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

  if (!learning_started_)
  {
    start_pose_ = odom_.pose.pose;
    const double start_angle = tf::getYaw(start_pose_.orientation);
    segment_.yaw = start_angle;
    //segment_.landmarks.clear();
    //landmarks_.clear();
    //segment_.distance = 0;
    learning_started_ = true;
  }
  
  processImage(msg, distance_from_start());
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": computation time: " << (ros::Time::now().toSec() - start_time.toSec()));
}

void AJockey::callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
  odom_ = *msg;
  has_odom_ = true;
}

/* Process an image and return the number of tracked features.
 *
 *  Return the number of potential features.
 *
 *  image[in] ROS image message.
 *  d[in] distance from the segment start in meters.
 */
size_t AJockey::processImage(const sensor_msgs::ImageConstPtr& image, const double d)
{
  image_processing_running_ = true;

  if (extract_features_ == NULL || match_descriptors_ == NULL)
  {
    ROS_ERROR_STREAM(ros::this_node::getName() << " extract_feature or match_descriptors function not set, doing nothing...");
    return 0;
  }

  vector<KeyPoint> keypoints;
  vector<Feature> descriptors;
  extract_features_(image, keypoints, descriptors);
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": number of detected features: " << keypoints.size());

  vector<size_t> used_features_idx;
  size_t newly_saved_landmarks_count = landmarks_.size();
  for (int i = landmarks_.size() - 1; i >= 0; --i)
  {
    if (descriptors.empty())
    {
      // If no feature was detected.
      ROS_DEBUG_STREAM(ros::this_node::getName() << ": no detected feature, waiting for next image");
      break;
    }
    vector<Feature> query_descriptors(1, landmarks_[i].landmark.descriptor);
    vector<vector<DMatch> > matches;
    match_descriptors_(query_descriptors, descriptors, matches);

    if (matches.empty())
    {
      /* ROS_ERROR_STREAM(ros::this_node::getName() << ": no match found"); */
      continue;
    }
    if (matches[0].size() < 2)
    {
      /* ROS_ERROR_STREAM(ros::this_node::getName() << ": not enough matches, found " << matches[0].size()); */
      continue;
    }

    if (matches[0][0].distance < matcher_equality_multiplication_ * matches[0][1].distance)
    {
      // Feature is still visible, update current landmark.
      landmarks_[i].landmark.dv = d;
      landmarks_[i].landmark.v.x = keypoints[matches[0][0].trainIdx].pt.x;
      landmarks_[i].landmark.v.y = keypoints[matches[0][0].trainIdx].pt.y;
      landmarks_[i].last_seen = image->header.stamp;
      // Save matched feature index to know which features to add as potential landmark.
      used_features_idx.push_back(matches[0][0].trainIdx);
    }
    else
    {
      // The feature is not available anymore, but was available long enough. Save it into segment_.
      if (std::abs(landmarks_[i].landmark.du - landmarks_[i].landmark.dv) > min_landmark_dist_)
      {
        segment_.landmarks.push_back(landmarks_[i].landmark);
      }
      landmarks_.erase(landmarks_.begin() + i);
    }
  }
  newly_saved_landmarks_count -= landmarks_.size();
  ROS_DEBUG("Number of landmarks added to segment: %zu", newly_saved_landmarks_count);
  ROS_DEBUG("Reused potential landmarks: %zu", used_features_idx.size());

  std::sort(used_features_idx.begin(), used_features_idx.end());
  for (size_t i = 0; i < keypoints.size(); ++i)
  {
    if (!std::binary_search(used_features_idx.begin(), used_features_idx.end(), i))
    {
      // Feature was not yet used, add it to potential landmarks.
      PotentialLandmark landmark(descriptors[i], image->header.stamp, keypoints[i].pt.x, keypoints[i].pt.y, d);
      landmarks_.push_back(landmark);
    }
  }

  size_t cleanedup_landmarks_count = landmarks_.size();
  for (int i = landmarks_.size() - 1; i >= 0; --i)
  {
    if ((image->header.stamp - landmarks_[i].last_seen) > max_landmark_age_)
    {
      landmarks_.erase(landmarks_.begin() + i);
    }
  }
  cleanedup_landmarks_count -= landmarks_.size();
  ROS_DEBUG("Number of cleanup potential landmarks: %zu", newly_saved_landmarks_count);

  ROS_DEBUG_STREAM(ros::this_node::getName() << ": number of potential landmarks: " << landmarks_.size());
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": number of learned landmarks: " << segment_.landmarks.size());
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": distance from the start [m]: " << d);

  image_processing_running_ = false;
  return landmarks_.size();
}

} // namespace featurenav_base
} // namespace lama



