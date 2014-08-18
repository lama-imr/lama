#ifndef _NJ_LASER_SHAPE_SIM_H_
#define _NJ_LASER_SHAPE_SIM_H_

#include <vector>
#include "laloc_utils.h"

namespace Lama {
namespace Laloc {

double shapeSimilarity(const std::vector<double>& r1, const std::vector<double>& r2);

std::vector<SPoint> filterRelevance(const std::vector<SPoint>& pts, const double maxR);

} // namespace Laloc
} // namespace Lama


#endif // _NJ_LASER_SHAPE_SIM_H_
