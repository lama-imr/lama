#ifndef ALJ_FEATURENAV_JOCKEY_H
#define ALJ_FEATURENAV_JOCKEY_H

#include <algorithm>
#include <vector>

#include <boost/scoped_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <featurenav_base/anjockey.h>
#include <featurenav_base/Feature.h>

namespace anj_featurenav
{

using std::vector;
using cv::KeyPoint;
using cv::DMatch;
using featurenav_base::Feature;

class Jockey
{
  public:

    Jockey(const std::string& name);

    std::string getLearningJockeyName() const;
    std::string getNavigatingJockeyName() const;

  private:

    void initDetectorFast();
    void initDetectorStar();
    void initDetectorOrb();
    void initDetectorMser();
    void initDetectorGftt();
    void initDetectorDense();
    void initDetectorSimpleblob();
    void initExtractorOrb();
    void initExtractorBrief();
    void initMatcherBruteforce();
    void initMatcherFlannbased();

    void extractFeatures(const sensor_msgs::ImageConstPtr& image, vector<KeyPoint>& keypoints, vector<Feature>& descriptors) const;
    void matchDescriptors(const vector<Feature>& query_descriptors, const vector<Feature>& train_descriptors, vector<vector<DMatch> >& matches) const;

    // ROS parameters, shown outside.
    std::string feature_detector_type_;
    std::string descriptor_extractor_type_;
    std::string descriptor_matcher_type_;

    // Internals.
    ros::NodeHandle private_nh_;
    std::string base_name_;  //!> The base name for learning and navigating jockeys.
    boost::scoped_ptr<lama::featurenav_base::ANJockey> anjockey_ptr_;
    boost::scoped_ptr<cv::FeatureDetector> feature_detector_;
    boost::scoped_ptr<cv::DescriptorExtractor> descriptor_extractor_;
    boost::scoped_ptr<cv::DescriptorMatcher> descriptor_matcher_;
    std::string feature_detector_code_;
    std::string descriptor_extractor_code_;
    std::string descriptor_matcher_code_;
    std::string map_interface_name_;  //!> Map interface name for Segment messages.
};
  
inline Feature rosFromCv(const cv::Mat& descriptor)
{
  Feature ros_descriptor;
  if ((descriptor.type() == CV_8U) ||
      (descriptor.type() == CV_16U))
  {
    std::copy(descriptor.begin<uint64_t>(), descriptor.end<uint64_t>(), std::back_inserter(ros_descriptor.udata));
  }
  else if ((descriptor.type() == CV_8S) ||
      (descriptor.type() == CV_16S) ||
      (descriptor.type() == CV_32S))
  {
    std::copy(descriptor.begin<int64_t>(), descriptor.end<int64_t>(), std::back_inserter(ros_descriptor.idata));
  }
  else if ((descriptor.type() == CV_32F) ||
      (descriptor.type() == CV_64F))
  {
    std::copy(descriptor.begin<double>(), descriptor.end<double>(), std::back_inserter(ros_descriptor.fdata));
  }
  else
  {
    ROS_ERROR("Unrecognized data type: %d", descriptor.type());
    throw ros::Exception("Unrecognized data type");
  }
  return ros_descriptor;
}

inline cv::Mat cvFromRos(const Feature& descriptor, const int type)
{
  if ((type == CV_8U) || (type == CV_16U))
  {
    cv::Mat cv_descriptor(descriptor.udata);
    return cv_descriptor;
  }
  else if ((type == CV_8S) || (type == CV_16S) || (type == CV_32S))
  {
    cv::Mat cv_descriptor(descriptor.idata);
    return cv_descriptor;
  }
  else if ((type == CV_32F) || (type == CV_64F))
  {
    cv::Mat cv_descriptor(descriptor.fdata);
    return cv_descriptor;
  }
  else
  {
    ROS_ERROR("Unrecognized data type: %d", type);
    throw ros::Exception("Unrecognized data type");
  }
}

inline void rosFromCv(const cv::Mat& cv_descriptors, vector<Feature>& ros_descriptors)
{
  ros_descriptors.clear();
  ros_descriptors.reserve(cv_descriptors.rows);
  for (int i = 0; i < cv_descriptors.rows; ++i)
  {
    const Feature ros_descriptor = rosFromCv(cv_descriptors.row(i));
    ros_descriptors.push_back(ros_descriptor);
  }
}

inline void cvFromRos(const vector<Feature>& ros_descriptors, cv::Mat& cv_descriptors, const int type)
{
  if (ros_descriptors.empty())
  {
    cv_descriptors.create(0, 0, type);
    return;
  }

  int size = std::max(ros_descriptors[0].udata.size(), std::max(ros_descriptors[0].idata.size(), ros_descriptors[0].fdata.size()));
  cv_descriptors.create(ros_descriptors.size(), size, type);

  for (size_t i = 0; i < ros_descriptors.size(); ++i)
  {
    cv_descriptors.row(i) = cvFromRos(ros_descriptors[i], type);
  }
  return;
}

} /* namespace anj_featurenav */  


#endif /* end of include guard: ALJ_FEATURENAV_JOCKEY_H */
