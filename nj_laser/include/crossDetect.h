#ifndef _LALOC_CROSSDETECT_H_
#define _LALOC_CROSSDETECT_H_

#include <vector>
#include "laloc_utils.h"


/**
  * routines for cross detection from laser scan. It is assumed
  * that two laser scanners are used to capture data 
  */

namespace Lama {
namespace Laloc {

/* SFrontier is a line segment through which the robot can go
 * p1 First point
 * p2 Second point, so that angle(r-p1, r-p2) is positive, where r is the laser base
 * width Segment length, i.e. width of free space
 * angle ?
 */
struct SFrontier {

	SFrontier(const SPoint &ip1, const SPoint &ip2, const double iwidth, const double iangle):
		p1(ip1),p2(ip2),width(iwidth),angle(iangle) {}

	SPoint p1, p2;
	double width;
	double angle;
};

/** @brief cross detection with frontiers.
  */
void cdPanoramatic3(
	const std::vector<double> &scan, const double rt, const double dt,
	const double minPhi, const double fov, const double maxFrontierAngle,
	std::vector<SFrontier> &frontiers);

/** @brief return center of a cross by finding largest circle. works fine
  */
void getCrossCenterVoronoi(const std::vector<SPoint> &pts, const double rt,
		const double dt, double &cx, double &cy, double &radius);

void getCrossCenterVoronoiWithKDTree(
		const vector<SPoint> &pts,  const double rt, const double dt,
		double &cx, double &cy, double &radius);

} // namespace Laloc
} // namespace Lama

#endif


