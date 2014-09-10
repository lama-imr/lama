
#ifndef _NJ_LASER_LALOC_UTILS_H_
#define _NJ_LASER_LALOC_UTILS_H_

#include <vector>
#include <string>

#include <sys/types.h>
#include <sys/resource.h>

#include <lama_common/point.h>

namespace lama {
namespace nj_laser {

using std::vector;
using lama::Point2;

vector<Point2> cutScan(const vector<double> &scan,
    const double maxPhi, const double rt);

/* save scan to a file
 * maxPhi is in degrees
 */
void saveScan(const char *name, const vector<double> &range, const double maxPhi=360.0);

void saveScan(const char *filename, const vector<Point2> &pts);

void saveDoubles(const char *filename, const vector<double> &values);

} // namespace nj_laser
} // namespace lama

#endif // _NJ_LASER_LALOC_UTILS_H_

