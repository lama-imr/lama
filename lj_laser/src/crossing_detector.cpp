/* Class to detect crossing center and exits.
 *
 * The crossing center is described by its position (x, y) and radius.
 * Exits are described by their angle (with the crossing center).
 */

#include <lj_laser/crossing_detector.h>

namespace lama {
namespace lj_laser {

CrossingDetector::CrossingDetector(const double frontier_width, const double max_frontier_angle) :
  frontier_width_(frontier_width),
  max_frontier_angle_(max_frontier_angle),
  max_frontier_dist_(0.0),
  max_frontier_dist_set_(false)
{
}

void CrossingDetector::setMaxFrontierDistance(const double value)
{
  if (value > 0)
  {
    max_frontier_dist_ = value;
    max_frontier_dist_set_ = true;
    return;
  }
  ROS_ERROR("Maximum frontier distance cannot be negative, ignoring");
}

/* Return the crossing center.
 *
 * scan[in] LaserScan
 * x[out] X-coordinate of the crossing center
 * y[out] Y-coordinate of the crossing center
 * r[out] radius of the crossing center
 */
bool CrossingDetector::crossingCenter(const sensor_msgs::LaserScan& scan, double& x, double& y, double& r) const
{
  bool exitFound = false;
  std::vector<Point> points = lama::scanToPolygon<Point>(scan);
  Delaunay triangulation;
  // TODO: add some points at frontiers.
  triangulation.insert(points.begin(), points.end());

  double max_radius = 0.0;

  for (Face_iterator face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face)
  {
    const Point c = triangulation.circumcenter(face);
    // TODO: Don't include c if c outside the polygon of all triangles.
    const Vertex v = *(face->vertex(0));
    const Point& p = v.point();
    const double circle_radius = std::sqrt((c[0] - p.x()) * (c[0] - p.x()) + (c[1] - p.y()) * (c[1] - p.y()));
    if (circle_radius > max_radius)
    {
      x = c[0];
      y = c[1];
      max_radius = circle_radius;
    }
  }
  r = max_radius;
  return exitFound;
}

/* Return frontiers.
 *
 * scan[in] LaserScan
 * frontiers[out] returned frontiers
 */
bool CrossingDetector::frontiers(const sensor_msgs::LaserScan& scan, vector<Frontier>& frontiers) const
{
  if (scan.ranges.size() < 2)
  {
    ROS_ERROR("Laser scan must have at least 2 points");
    return false;
  }

  double frontier_dist = max_frontier_dist_;
  if (!max_frontier_dist_set_)
  {
    frontier_dist = 0.9 * (*std::max_element(scan.ranges.begin(), scan.ranges.end()));
  }

  vector<double> filtScan(scan.ranges.size());
  vector<int> angleNumber(scan.ranges.size());

  // Filter out laser beams longer than frontier_dist.
  for(size_t i = 0; i < scan.ranges.size(); i++)
  {
    if (scan.ranges[i] < frontier_dist)
    {
      filtScan.push_back(scan.ranges[i]);
      angleNumber.push_back(i);
    }
  }

  if (filtScan.size() == 0)
  {
    ROS_ERROR("All scan ranges are longer than threshold %f", frontier_dist);
    return false;
  }

  double phiResolution = scan.angle_increment;
  double aAngle;
  double bAngle;
  geometry_msgs::Point a;
  geometry_msgs::Point b;
  aAngle = scan.angle_min + angleNumber[0] * phiResolution;
  a.x = filtScan[0] * std::cos(aAngle);
  a.y = filtScan[0] * std::sin(aAngle);
  Frontier frontier;

  double dist2;
  double dt2 = frontier_width_ * frontier_width_;
  frontiers.clear();

  for(size_t i = 1; i < filtScan.size() + 1; i++)
  {
    const size_t j = i % filtScan.size();
    bAngle = scan.angle_min + angleNumber[j] * phiResolution;
    b.x = filtScan[j] * std::cos(bAngle);
    b.y = filtScan[j] * std::sin(bAngle);
    dist2 = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);

    if (dist2 > dt2)
    {
      double sx = (a.x + b.x) / 2.0;
      double sy = (a.y + b.y) / 2.0;

      double distToFrontierCenter = std::sqrt(sx * sx + sy * sy);
      double dotProductFrontierSxSy = (b.x - a.x) * sx + (b.y - a.y) * sy;
      double dist = std::sqrt(dist2);
      double frontierAngleWithSxSy = std::acos(dotProductFrontierSxSy / dist / distToFrontierCenter);
      if (std::fabs(M_PI_2 - frontierAngleWithSxSy) < max_frontier_angle_)
      {
        frontier.p1 = a;
        frontier.p2 = b;
        frontier.width = dist;
        frontier.angle = std::atan2(sy, sx);
        frontiers.push_back(frontier);
      }
    }
    a.x = b.x;
    a.y = b.y;
    aAngle = bAngle;
  }

  return (frontiers.size() != 0);
}

} // namespace nj_laser
} // namespace lama
