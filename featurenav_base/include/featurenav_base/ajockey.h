#ifndef FEATURENAV_BASE_AJOCKEY_H
#define FEATURENAV_BASE_AJOCKEY_H

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <lama_jockeys/learning_jockey.h>

#include <featurenav_base/potential_landmark.h>
#include <featurenav_base/typedef.h>
#include <featurenav_base/Landmark.h>
#include <featurenav_base/Segment.h>
#include <featurenav_base/SetSegment.h>

namespace lama {
namespace featurenav_base {

using std::vector;
using cv::Mat;
using cv::KeyPoint;
using cv::DMatch;

class AJockey : public LearningJockey
{
  public:

    AJockey(const std::string& name, const std::string& segment_interface_name, const std::string& segment_setter_name);

    void setExtractFeaturesFunction(feature_extractor_function_ptr f) {extract_features_ = f;}
    void setDescriptorMatcherFunction(descriptor_matcher_function_ptr f) {match_descriptors_ = f;}

    can_do_function_ptr canDo;
    start_do_function_ptr startDo;

  private:

    virtual void onStartLearn();
    virtual void onStopLearn();
    virtual void onInterrupt();
    virtual void onContinue();

    void reset();
    size_t processImage(const sensor_msgs::ImageConstPtr& image);

    /* Return the distance since we started learning.
     */
    inline double distance_from_start()
    {
      if (!has_odom_)
      {
        return 0.0;
      }
      const double dx = odom_.pose.pose.position.x - start_pose_.position.x;
      const double dy = odom_.pose.pose.position.y - start_pose_.position.y;
      return std::sqrt(dx * dx + dy * dy);
    }

    /* Return a DescriptorLink with the given id
     */
    inline lama_interfaces::DescriptorLink segmentDescriptorLink(const int32_t id)
    {
      lama_interfaces::DescriptorLink descriptor_link;
      descriptor_link.descriptor_id = id;
      descriptor_link.interface_name = segment_interface_name_;
      return descriptor_link;
    }

    void callback_image(const sensor_msgs::ImageConstPtr& msg);
    void callback_odom(const nav_msgs::OdometryConstPtr& msg);

    // Publisher and subscribers.
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_handler_;
    ros::Subscriber odom_handler_;

    // Client proxies.
    ros::ServiceClient segment_setter_proxy_;

    // ROS parameters, shown outside.
    double matcher_max_relative_distance_;  //!> A potential descriptor is visible in the new image if
                                            //!> the distance to the best-match descriptor is smaller than
                                            //!> the distance to the second best match multiplied by this factor.
    double min_landmark_dist_;  //!> A landmark is saved if it was visible on at least such a traveled distance.

    // Hard-coded parameters.
    static const ros::Duration max_odom_age_;
    static const ros::Duration max_landmark_age_;

    // Internals.
    std::string segment_interface_name_;  //!> Name of the map interface for segments.
    std::string segment_setter_name_;  //!> Name of the service to write segments into the map.
    feature_extractor_function_ptr extract_features_;
    descriptor_matcher_function_ptr match_descriptors_;
    nav_msgs::Odometry odom_;  //!> Last received odometry message.
    bool has_odom_;
    bool image_processing_running_;  //!> true when treating an image.
    geometry_msgs::Pose start_pose_;  //!> Pose when learning started.
    ::featurenav_base::Segment segment_;
    std::vector<PotentialLandmark> landmarks_; //!> Potential landmarks (features seen once).
};

} // namespace featurenav_base
} // namespace lama

#endif // FEATURENAV_BASE_AJOCKEY_H

