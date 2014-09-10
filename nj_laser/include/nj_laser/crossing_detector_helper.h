#ifndef _NJ_LASER_CROSSING_DETECTOR_HELPER_H_
#define _NJ_LASER_CROSSING_DETECTOR_HELPER_H_

#include <vector>

#include <sensor_msgs/LaserScan.h>

#include <lama_common/point.h>
#include <lama_msgs/Frontier.h>

#include <nj_laser/shape_sim.h>
#include <nj_laser/laloc_utils.h>

/**
  * routines for cross detection from laser scan. It is assumed
  * that two laser scanners are used to capture data 
  */

namespace lama {
namespace nj_laser {

using std::vector;
using lama::Point2;
using sensor_msgs::LaserScan;
using lama_msgs::Frontier;

/** @brief cross detection with frontiers.
  */
void cdPanoramatic3(const LaserScan& scan,
    const double rt, const double dt, const double maxFrontierAngle,
    vector<Frontier> &frontiers);

/** @brief return center of a cross by finding largest circle. works fine
  */
void getCrossCenterVoronoi(const std::vector<Point2> &pts, const double rt,
    const double dt, double &cx, double &cy, double &radius);

void getCrossCenterVoronoiWithKDTree(
    const std::vector<Point2> &pts,  const double rt, const double dt,
    double &cx, double &cy, double &radius);

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_CROSSING_DETECTOR_HELPER_H_


