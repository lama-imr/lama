/* Action server for the memory-less navigating jockey based on LaserScan.
 *
 * The role of this jockey is to travel to the next crossing.
 * The action is done when the robot reaches the crossing center.
 *
 * Interaction with the map:
 * - Getter: none
 * - Setter: none.
 *
 * Subscribers (other than map-related):
 * - sensor_msg::LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - visualization_msgs::Marker, "~/crossing_marker", a sphere at the crossing center.
 * - visualization_msgs::Marker, "~/exits_marker", lines from crossing center towards exits.
*/

#ifndef _NJ_LASER_JOCKEY_H_
#define _NJ_LASER_JOCKEY_H_

#include <sensor_msgs/LaserScan.h>

#include <lama_jockeys/navigating_jockey.h>
#include <lama_msgs/crossing_visualization.h>
#include <goto_crossing/crossing_goer.h>
#include <crossing_detector/laser_crossing_detector.h>

#include <nj_laser/visualization.h>

namespace lama {
namespace nj_laser {

class Jockey : public lama::NavigatingJockey
{
  public:

    Jockey(std::string name, const double frontier_width);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);

  private:

    // Subscribers and publishers.
    ros::Subscriber laserHandler_;
    ros::Publisher pub_crossing_marker_;
    ros::Publisher pub_exits_marker_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_place_profile_;
    ros::Publisher pub_crossing_;

    // Parameters shown outside.
    double max_frontier_dist_;

    // Internals.
    bool has_scan_;
    lama_msgs::Crossing crossing_;  //!> Crossing descriptor from LaserScan.
    lama::crossing_detector::LaserCrossingDetector crossing_detector_;
    goto_crossing::CrossingGoer crossing_goer_;
};

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_JOCKEY_H_

