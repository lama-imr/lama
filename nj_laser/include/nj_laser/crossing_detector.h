/*
 * localization module based on laser detection of a crossing
 */

#ifndef _NJ_LASER_CROSSING_DETECTOR_H_
#define _NJ_LASER_CROSSING_DETECTOR_H_

#include <math.h>
#include <algorithm>
#include <list>
#include <sstream>
#include <iostream>
#include <vector>
#include <utility>  // for std::pair

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <lama_common/frontier.h>

#include <nj_laser/laloc_utils.h>
#include <nj_laser/crossing_detector_helper.h>

namespace lama {
namespace nj_laser {

using lama::Frontier;

class CrossingDetector
{
	public:

    CrossingDetector();
    ~CrossingDetector();

    void setDescriptor(const sensor_msgs::LaserScan& scan);

    void crossDetect(const sensor_msgs::LaserScan& scan);

    double getCrossCenterX() const;
    double getCrossCenterY() const;
    double getCrossRadius() const;

    int getNumExits() const;
    std::vector<double> getExitAngles() const;
    double getExitAngle(const int i) const;
    double getExitWidth(const int i) const;

    const std::vector<double>& getDescriptor() const;
    const std::vector<double>& getCrossDescriptor() const;

    int getDescriptorSize() const;

  private:

    // distance threshold .. longer lines are considered frontiers
    const static double frontier_width;
    // max. allowed frontier angle in radians (0 means angle between line from
    // robot to frontier middle and frontier is 90 deg).
    const static double max_frontier_angle;

    // fft descriptor of a place
    std::vector<double> actDescriptor;

    // [cx,cy,cr, angle1, angle2 ... ]
    // descriptor of a cross
    // if no cros .. cx=cy=cr=0
    std::vector<double> actCrossdescriptor;
    std::vector<Frontier> actFrontiers;
};

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_CROSSING_DETECTOR_H_

