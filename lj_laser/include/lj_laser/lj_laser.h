/* Action server for lj_laser node.
 */

#ifndef _LJ_LASER_LJ_LASER_H_
#define _LJ_LASER_LJ_LASER_H_

#include <sensor_msgs/LaserScan.h>

#include <lama_common/frontier.h>
#include <lama_interfaces/SetVectorLaserScan.h>
#include <lama_interfaces/SetVectorDouble.h>
#include <lama_jockeys/localizing_jockey.h>

#include <lj_laser/crossing_detector.h>

namespace lama {
namespace lj_laser {

class LJLaser : public lama::LocalizingJockey
{
  public:

    LJLaser(std::string name, const double frontier_width, const double max_frontier_angle=0.785);

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

#endif // _LJ_LASER_LJ_LASER_H_
