/*
 * Localizing Jockey based on LaserScan.
 *
 * The role of this jockey is to get the similarity of the LaserScan
 * descriptors of all vertices with the current LaserScan.
 * The action is done when the similarities are computed.
 * Implemented actions:
 * - GET_VERTEX_DESCRIPTOR: return the LaserScan, the PlaceProfile and the computed Crossing
 * - GET_SIMILARITY: return the dissimilarity based on LaserScan
 *
 * Interaction with the map (created by this jockey):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter/Setter: VectorLaserScan, jockey_name + "_laser_descriptor"
 *
 * Interaction with the map (created by other jockeys):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter: VectorLaserScan, "laser_descriptor"
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs::LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 * - nav_msgs::Pose, "~/pose", robot pose
 *
 * Services used (other than map-related):
 * - service type, server default name, description
 * - polygon_matcher::PolygonSimilarity, "~/similarity_server", used to
 *    compare all known places with the current place
 *
 * Parameters:
 * - name, type, default name, description
 */

#ifndef _LJ_LASER_JOCKEY_H_
#define _LJ_LASER_JOCKEY_H_

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_common/polygon.h>  // for scanToPolygon
#include <lama_interfaces/ActOnMap.h>
#include <lama_interfaces/MapAction.h>
#include <lama_interfaces/AddInterface.h>
#include <lama_interfaces/GetVectorLaserScan.h>
#include <lama_interfaces/SetVectorLaserScan.h>
#include <lama_interfaces/SetVectorDouble.h>
#include <lama_msgs/Frontier.h>
#include <lama_msgs/SetCrossing.h>
#include <lama_jockeys/localizing_jockey.h>
#include <polygon_matcher/PolygonSimilarity.h>

#include <crossing_detector/laser_crossing_detector.h>

namespace lama {
namespace lj_laser {

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

    void setSimilarityServerName(std::string name) {similarity_server_name_ = name;}

  protected:

    bool data_received_;

    // Reception and storage of LaserScan.
    ros::Subscriber laserHandler_;
    ros::Time scan_reception_time_;
    sensor_msgs::LaserScan scan_;

    // Reception and Sending of LaserScan and Crossing descriptors.
    std::string laser_interface_name_;
    ros::ServiceClient laser_descriptor_getter_;
    ros::ServiceClient laser_descriptor_setter_;
    std::string crossing_interface_name_;
    ros::ServiceClient crossing_descriptor_setter_;

    // Similarity server.
    std::string similarity_server_name_;
    ros::ServiceClient similarity_server_;

    lama::crossing_detector::LaserCrossingDetector crossing_detector_;

  private:

    void initMapLaserInterface();
    void initMapCrossingInterface();
    void getData();
    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);
};


} // namespace lj_laser
} // namespace lama

#endif // _LJ_LASER_JOCKEY_H_
