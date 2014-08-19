#ifndef _NJ_LASER_SHAPE_SIM_H_
#define _NJ_LASER_SHAPE_SIM_H_

#include <vector>

#include <nj_laser/laloc_utils.h>

namespace lama {
namespace Laloc {

double shapeSimilarity(const std::vector<double>& r1, const std::vector<double>& r2);

std::vector<lama::Point2> filterRelevance(const std::vector<lama::Point2>& pts, const double maxR);

} // namespace Laloc
} // namespace lama


#endif // _NJ_LASER_SHAPE_SIM_H_
