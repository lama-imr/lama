#ifndef _NJ_LASER_CROSSDETECT_H_
#define _NJ_LASER_CROSSDETECT_H_

#include <vector>

#include <lama_common/point.h>
#include <lama_common/frontier.h>
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
using lama::Frontier;

/** @brief cross detection with frontiers.
  */
void cdPanoramatic3(
    const std::vector<double> &scan, const double rt, const double dt,
    const double minPhi, const double fov, const double maxFrontierAngle,
    std::vector<Frontier> &frontiers);

/** @brief return center of a cross by finding largest circle. works fine
  */
void getCrossCenterVoronoi(const std::vector<Point2> &pts, const double rt,
    const double dt, double &cx, double &cy, double &radius);

void getCrossCenterVoronoiWithKDTree(
    const std::vector<Point2> &pts,  const double rt, const double dt,
    double &cx, double &cy, double &radius);

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_CROSSDETECT_H_


