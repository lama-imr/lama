/*
 * Localizing Jockey based on LaserScan.
 *
 * Localizing Jockey based on LaserScan.
 * - description of general role
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

#include <sensor_msgs/LaserScan.h>

#include <lama_common/frontier.h>
#include <lama_interfaces/SetVectorLaserScan.h>
#include <lama_interfaces/SetVectorDouble.h>
#include <lama_jockeys/localizing_jockey.h>

#include <lj_laser/crossing_detector.h>

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

  private:

    void getLaserScan();
    void handleLaser(const sensor_msgs::LaserScan msg);

    ros::Subscriber laserHandler_;

    bool scan_received_;
    sensor_msgs::LaserScan scan_;
    CrossingDetector crossing_detector_;
};

} // namespace lj_laser
} // namespace lama

#endif // _LJ_LASER_JOCKEY_H_
