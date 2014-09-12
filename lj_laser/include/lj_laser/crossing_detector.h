#ifndef _LJ_LASER_CROSSING_DETECTOR_H_
#define _LJ_LASER_CROSSING_DETECTOR_H_

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

#include <lama_common/point.h>
#include <lama_common/polygon.h>
#include <lama_msgs/Frontier.h>

namespace lama {
namespace lj_laser {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef Delaunay::Face_iterator Face_iterator;
typedef Delaunay::Vertex Vertex;
typedef K::Point_2 Point;

using std::vector;
using lama_msgs::Frontier;

class CrossingDetector
{
  public:

    CrossingDetector(const double frontier_width, const double max_frontier_angle=0.785);

    bool crossingCenter(const sensor_msgs::LaserScan& scan, double& x, double& y, double& r) const;
    bool frontiers(const sensor_msgs::LaserScan& scan, vector<Frontier>& frontiers) const;

    double getFrontierWidth() const {return frontier_width_;}
    void setFrontierWidth(const double value) {frontier_width_ = std::fabs(value);}

    double getMaxFrontierAngle() const {return max_frontier_angle_;}
    void setMaxFrontierAngle(const double value) {max_frontier_angle_ = std::fabs(value);}

    double getMaxFrontierDistance() const {return max_frontier_dist_;}
    void setMaxFrontierDistance(const double value);
    
  private:

    double frontier_width_;  //!> Min. frontier width (m).
    double max_frontier_angle_;  //!> Max. angle between frontier and line from robot to frontier (rad).
                                 //!> 0 means that the angle between the line from robot to frontier middle
                                 //!> and the frontier is 90 deg.
    double max_frontier_dist_;  //!> Max range for a laser beam to be considered infinite.

    bool max_frontier_dist_set_;  //!> true if the user has called setMaxFrontierDistance.
};

} // namespace lj_laser
} // namespace lama

#endif // _LJ_LASER_CROSSING_DETECTOR_H_
