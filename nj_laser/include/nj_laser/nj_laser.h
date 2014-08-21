/* Memory-less navigating jockey based on LaserScan
*/

#ifndef _NJ_LASER_NJ_LASER_H_
#define _NJ_LASER_NJ_LASER_H_

#include <sensor_msgs/LaserScan.h>

#include <lama_interfaces/navigating_jockey.h>

#include <nj_laser/claloc.h>
#include <nj_laser/visualization.h>

namespace lama {
namespace nj_laser {

class NJLaser : public lama::interfaces::NavigatingJockey
{
  public:

    NJLaser(std::string name);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

  private:

    void handleLaser(const sensor_msgs::LaserScan msg);

    ros::Subscriber laserHandler_;
    ros::Publisher pub_crossing_marker_;
    ros::Publisher pub_exits_marker_;
    ros::Publisher pub_twist_;
    lama::nj_laser::CLaloc cross_detector;
};

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_NJ_LASER_H_

