#include <nj_laser/crossing_detector.h>

namespace lama {
namespace nj_laser {

using std::list;
using std::vector;
using std::cerr;
using std::pair;
using std::stringstream;

const double CrossingDetector::frontier_width = 0.7;
const double CrossingDetector::max_frontier_angle = 45 * M_PI / 180.0;

/** notes:
  * actDescriptor contains only RANGES_FROM_RANGE_FINDER
  * but ! received vertex contain this descriptor plus another data, such as
  * resolution of range finder or number of angles per scan
  * so: 
  * working with actDescriptor .. it is ok
  */

CrossingDetector::CrossingDetector()
{
  ROS_INFO("CrossingDetector: distance threshold frontier_width = %.3f", frontier_width);
  ROS_INFO("CrossingDetector: max_frontier_angle = %.3f", max_frontier_angle);
}

CrossingDetector::~CrossingDetector()
{
  actDescriptor.clear();
  actCrossdescriptor.clear();
}

void CrossingDetector::setDescriptor(const sensor_msgs::LaserScan& scan)
{
  crossDetect(scan);
  actDescriptor.clear();
  for (std::vector<float>::const_iterator it = scan.ranges.begin(); it != scan.ranges.end(); ++it)
    actDescriptor.push_back(*it);
}

const vector<double>& CrossingDetector::getDescriptor() const
{
  return actDescriptor;
}

const vector<double>& CrossingDetector::getCrossDescriptor() const
{
  return actCrossdescriptor;
}

/* Compute the crossing centers
 *
 * scan[in] sensor_msgs::LaserScan
 */
void CrossingDetector::crossDetect(const sensor_msgs::LaserScan& scan)
{ 
  ros::Time t1;
  ros::Time t2;
  ros::Duration dt;


  double maxRange = *std::max_element(scan.ranges.begin(), scan.ranges.end());
  double rtOpt = 0.9 * maxRange;

  vector<Frontier> frontiers;
  t1 = ros::Time::now();
  cdPanoramatic3(scan, rtOpt, frontier_width, max_frontier_angle, frontiers);
  t2 = ros::Time::now();
  dt = t2 - t1;
  ROS_DEBUG("Time of detecting exits from cross: %.3f s", dt.toSec());

  vector<double> res;

  if (frontiers.size() == 0)
  {
    res.push_back(0);
    res.push_back(0);
    res.push_back(-1);
    double m = -1;
    int mi = 0;
    for(int k = 0; k < scan.ranges.size(); k++)
    {
      if (scan.ranges[k] > m || m == -1)
      {
        mi = k;
        m = scan.ranges[k];
      }
    }
    res.push_back(mi * scan.angle_max / scan.ranges.size());
  }
  else
  {
    std::vector<double> ranges(scan.ranges.size());
    for (std::vector<float>::const_iterator it = scan.ranges.begin(); it != scan.ranges.end(); ++it)
      ranges.push_back(*it);

    double cx, cy, cr;
    t1 = ros::Time::now();
    //getCrossCenterVoronoi(cutScan(ranges,scan.angle_max,rtOpt),rtOpt,frontier_width,cx,cy,cr);
    getCrossCenterVoronoiWithKDTree(cutScan(ranges, scan.angle_max, rtOpt),rtOpt,frontier_width,cx,cy,cr);
    t2 = ros::Time::now();
    dt = t2 - t1;
    ROS_DEBUG("Time of cross center search: %.3f s", dt.toSec());
    res.push_back(-cy); // TODO: explain why -cy (minus sign and y before x)
    res.push_back(-cx);
    res.push_back(cr);
    for(int i = 0; i < frontiers.size(); i++)
    {
      res.push_back(frontiers[i].angle);
    }
  }
  actFrontiers = frontiers;
  actCrossdescriptor = res;
}

double CrossingDetector::getCrossCenterX() const
{
  if (actCrossdescriptor.size() > 0)
    return actCrossdescriptor[0];
  return 0;
}

double CrossingDetector::getCrossCenterY() const
{
  if (actCrossdescriptor.size() > 1)
    return actCrossdescriptor[1];
  return 0;
}

double CrossingDetector::getCrossRadius() const
{
  if (actCrossdescriptor.size() > 2)
    return actCrossdescriptor[2];
  return 0;
}

int CrossingDetector::getNumExits() const
{
  return std::max(0, (int)(actCrossdescriptor.size() - 3));
}

std::vector<double> CrossingDetector::getExitAngles() const
{
	std::vector<double> exitAngles;
	for (size_t i = 3; i < actCrossdescriptor.size(); ++i)
	{
		exitAngles.push_back(actCrossdescriptor[i]);
	}
	return exitAngles;
}

double CrossingDetector::getExitAngle(const int i) const
{
  if (i >=0 && i < actFrontiers.size())
  {
    return actFrontiers[i].angle;
  }
  return 0;
}

double CrossingDetector::getExitWidth(const int i) const
{
  if (i >= 0 && i < actFrontiers.size())
  {
    return actFrontiers[i].width;
  }
  return -1;
}

int CrossingDetector::getDescriptorSize() const
{
  return actDescriptor.size();
}

} // namespace nj_laser
} // namespace lama


