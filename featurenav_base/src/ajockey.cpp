#include <featurenav_base/ajockey.h>

namespace lama {
namespace featurenav_base {

const ros::Duration AJockey::max_odom_age_ = ros::Duration(0.5);
const double AJockey::matcher_equality_multiplication_ = 0.8;
const double AJockey::min_landmark_dist_ = 0.02;
const ros::Duration AJockey::max_landmark_age_ = ros::Duration(5.0);

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

  ROS_DEBUG_STREAM("Waiting for service \"" << segment_setter_name_ << "\"");
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
  ROS_DEBUG("Received ON_STOP_LEARN");

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

  ROS_DEBUG("Waiting for processImage to finish");

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

  ROS_DEBUG("processImage finished");

  segment_.distance = distance_from_start();
  /* for (size_t i = 0; i < landmarks_.size(); ++i) */
  /* { */
  /*   segment_.landmarks.push_back(landmarks_[i].landmark); */
  /* } */

  // Save the segment into the database and return its id.
  ROS_INFO("Saving a segment with %zu landmarks",segment_.landmarks.size());
  ::featurenav_base::SetSegment segment_setter;
  segment_setter.request.descriptor = segment_;
  if (!segment_setter_proxy_.call(segment_setter))
  {
    ROS_ERROR("Failed to add Segment to the map");
    server_.setAborted();
    return;
  }
  ROS_DEBUG("Added Segment with id %d", segment_setter.response.id);
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
  ROS_DEBUG("Computation time: %.3f", (ros::Time::now().toSec() - start_time.toSec()));
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
    ROS_ERROR("Extract_feature or match_descriptors function not set, doing nothing...");
    return 0;
  }

  vector<KeyPoint> keypoints;
  vector<Feature> descriptors;
  extract_features_(image, keypoints, descriptors);

  if (descriptors.empty())
  {
    // If no feature was detected.
    ROS_DEBUG("No detected feature, waiting for next image");
    return 0;
  }

  if (keypoints.size() != descriptors.size())
  {
    ROS_ERROR("keypoints and descriptors differ in call of extract_features function");
    return 0;
  }

  ROS_DEBUG_STREAM("Number of detected features: " << keypoints.size());

  // Match descriptors.
  vector<Feature> query_descriptors;
  query_descriptors.reserve(landmarks_.size());
  for (size_t i = 0; i < landmarks_.size(); ++i)
  {
    query_descriptors.push_back(landmarks_[i].landmark.descriptor);
  }
  vector<vector<DMatch> > matches;
  match_descriptors_(query_descriptors, descriptors, matches);

  if (matches.empty())
  {
    ROS_ERROR_STREAM("No match found");
  }

  vector<size_t> used_features_idx;
  vector<size_t> landmark_to_save;
  vector<size_t> landmark_to_delete;
  for (size_t i = 0; i < matches.size();  ++i)
  {
    if (matches[i].size() < 2)
    {
      ROS_DEBUG("Not enough matches, found %zu", matches[i].size());
      continue;
    }

    const size_t landmark_idx = matches[i][0].queryIdx;
    const size_t feature_idx = matches[i][0].trainIdx;

    if (matches[i][0].distance < matcher_equality_multiplication_ * matches[i][1].distance)
    {
      // Feature is still visible, update corresponding landmark.
      landmarks_[landmark_idx].landmark.dv = d;
      landmarks_[landmark_idx].landmark.v.x = keypoints[feature_idx].pt.x;
      landmarks_[landmark_idx].landmark.v.y = keypoints[feature_idx].pt.y;
      landmarks_[landmark_idx].last_seen = image->header.stamp;
      // Save matched feature index to know which features to add as potential landmark.
      used_features_idx.push_back(feature_idx);
    }
    else
    {
      // The feature is not available anymore, but was available before.
      if (std::abs(landmarks_[landmark_idx].landmark.du - landmarks_[landmark_idx].landmark.dv) > min_landmark_dist_)
      {
        // The feature was visible over a certain distance, save it into segment_.
        landmark_to_save.push_back(landmark_idx);
      }
      // Delete the segment, whether it is used or not.
      landmark_to_delete.push_back(landmark_idx);
    }
  }

  std::sort(landmark_to_save.begin(), landmark_to_save.end());
  int last_index = -1;
  for (int i = 0; i < landmark_to_save.size(); i++)
  {
    if (i != last_index)
    {
      segment_.landmarks.push_back(landmarks_[landmark_to_save[i]].landmark);
    }
    last_index = i;
  }

  std::sort(landmark_to_delete.begin(), landmark_to_delete.end());
  for (int i = landmark_to_delete.size() - 1; i <= 0; --i)
  {
    if (i < landmarks_.size())
    {
      landmarks_.erase(landmarks_.begin() + landmark_to_delete[i]);
    }
  }

  ROS_DEBUG("Number of landmarks added to segment: %zu", landmark_to_save.size());
  ROS_DEBUG("Reused potential landmarks: %zu", used_features_idx.size());

  // Add new features as potential landmarks.
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

  // Clean up old landmarks.
  size_t cleanedup_landmarks_count = landmarks_.size();
  for (int i = landmarks_.size() - 1; i >= 0; --i)
  {
    if ((image->header.stamp - landmarks_[i].last_seen) > max_landmark_age_)
    {
      landmarks_.erase(landmarks_.begin() + i);
    }
  }
  cleanedup_landmarks_count -= landmarks_.size();
  ROS_DEBUG("Number of cleaned-up potential landmarks: %zu", cleanedup_landmarks_count);

  ROS_DEBUG("Number of potential landmarks: %zu", landmarks_.size());
  ROS_DEBUG("Number of learned landmarks: %zu", segment_.landmarks.size());
  ROS_DEBUG("Distance from the start [m]: %.3f", d);

  image_processing_running_ = false;
  return landmarks_.size();
}

} // namespace featurenav_base
} // namespace lama



