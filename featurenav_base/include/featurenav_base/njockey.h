#ifndef FEATURENAV_BASE_NJOCKEY_H
#define FEATURENAV_BASE_NJOCKEY_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <lama_jockeys/navigating_jockey.h>

#include <featurenav_base/typedef.h>

namespace lama {
namespace featurenav_base {

class NJockey : public NavigatingJockey
{
  public:

    NJockey(const std::string& name);

    void setExtractFeaturesFunction(feature_extractor_function_ptr f) {extract_features_ = f;}
    void setDescriptorMatcherFunction(descriptor_matcher_function_ptr f) {match_descriptors_ = f;}

    can_do_function_ptr canDo;
    start_do_function_ptr startDo;

  private:

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void callback_image(const sensor_msgs::ImageConstPtr& msg);

    // Publisher and subscribers.
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_handler_;

    feature_extractor_function_ptr extract_features_;
    descriptor_matcher_function_ptr match_descriptors_;

    // Internals.
    bool segment_end_reached_;
};

} // namespace featurenav_base
} // namespace lama

#endif // FEATURENAV_BASE_NJOCKEY_H

