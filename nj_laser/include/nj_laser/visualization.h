#ifndef _NJ_LASER_VISUALIZATION_H_
#define _NJ_LASER_VISUALIZATION_H_

#include <visualization_msgs/Marker.h>

visualization_msgs::Marker crossingMarker(const std::string& frame_id, const double x, const double y, const double radius);

visualization_msgs::Marker exitsMarker(const std::string& frame_id, const std::vector<double>& angles, const double length);

#endif // #ifndef _NJ_LASER_VISUALIZATION_H_
