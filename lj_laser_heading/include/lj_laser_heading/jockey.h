/* Action server for lj_laser_heading node.
 */

#ifndef _LJ_LASER_HEADING_JOCKEY_H_
#define _LJ_LASER_HEADING_JOCKEY_H_

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <lama_common/frontier.h>
#include <lama_interfaces/ActOnMap.h>
#include <lama_interfaces/MapAction.h>
#include <lama_interfaces/AddInterface.h>
#include <lama_interfaces/GetVectorLaserScan.h>
#include <lama_interfaces/SetVectorLaserScan.h>
#include <lama_interfaces/SetVectorDouble.h>
#include <lama_jockeys/localizing_jockey.h>
#include <polygon_matcher/PolygonSimilarity.h>

#include <lj_laser/crossing_detector.h>

namespace lama {
namespace lj_laser_heading {

class Jockey : public lama::LocalizingJockey
{
  public:

    Jockey(std::string name, const double frontier_width, const double max_frontier_angle=0.785);

    virtual void onGetVertexDescriptor();
    virtual void onGetEdgesDescriptors();
    virtual void onLocalizeInVertex();
    virtual void onLocalizeEdge();
    virtual void onGetSimilarity();
    // virtual void onInterrupt();
    // virtual void onContinue();

    void set_similarity_server_name(std::string name) {similarity_server_name_ = name;}

  private:

    void getData();
    void handleLaser(const sensor_msgs::LaserScan msg);
    void handlePose(const geometry_msgs::Pose msg);

    void rotateScan();

    // Reception and storage of LaserScan and Pose.
    std::string laser_interface_name_;
    ros::Subscriber laserHandler_;
    ros::ServiceClient laser_descriptor_getter_;
    ros::Subscriber poseHandler_;
    bool data_received_;
    ros::Time scan_reception_time_;
    ros::Time pose_reception_time_;
    const static ros::Duration max_data_time_delta_;  //!> Max time interval between reception of scan_ and pose_.
    sensor_msgs::LaserScan scan_;
    geometry_msgs::Pose pose_;  //!> Only the heading information will be used.

    // Similarity server.
    std::string similarity_server_name_;
    ros::ServiceClient similarity_server_;

    lama::lj_laser::CrossingDetector crossing_detector_;
};

} // namespace lj_laser_heading
} // namespace lama

#endif // _LJ_LASER_HEADING_JOCKEY_H_
