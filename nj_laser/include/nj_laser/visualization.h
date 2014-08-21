#ifndef _NJ_LASER_VISUALIZATION_H_
#define _NJ_LASER_VISUALIZATION_H_

#include <string>

#include <visualization_msgs/Marker.h>

namespace lama {
namespace nj_laser {

visualization_msgs::Marker crossingMarker(const std::string& frame_id, const double x, const double y, const double radius);

visualization_msgs::Marker exitsMarker(const std::string& frame_id, const std::vector<double>& angles, const double length);

} // namespace nj_laser
} // namespace lama

#endif // #ifndef _NJ_LASER_VISUALIZATION_H_