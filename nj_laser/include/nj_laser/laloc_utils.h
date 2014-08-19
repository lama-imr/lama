
#ifndef _NJ_LASER_LALOC_UTILS_H_
#define _NJ_LASER_LALOC_UTILS_H_

#include <vector>
#include <string>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>

#include <lama_common/point.h>

namespace lama {
namespace Laloc {

using std::vector;
using lama::Point2;

typedef struct _IPoint
{
  int x,y;
} IPoint;

class Segment
{
  public:

    Segment() {};
    Segment(const double _x1, const double _y1,
        const int _i1, const double _x2,
        const double _y2, const int _i2) :
      x1(_x1), y1(_y1), from(_i1), x2(_x2), y2(_y2), to(_i2)
  {
  }

    Segment(const Point2 &p1, const int i1,
        const Point2 &p2, const int i2) :
      x1(p1.x), y1(p1.y), from(i1), x2(p2.x), y2(p2.y), to(i2)
  {
  }

    double x1;
    double y1;	
    int from;
    double x2;
    double y2;
    int to;
};

void setRandom();

vector<Point2> cutScan(const vector<double> &scan,
    const double maxPhi, const double rt);

/* save scan to a file
 * maxPhi is in degrees
 */
void saveScan(const char *name, const vector<double> &range, const double maxPhi=360.0);

void saveScan(const char *filename, const vector<Point2> &pts);

void saveDoubles(const char *filename, const vector<double> &values);

double getTime(struct rusage one, struct rusage two);
void getTime(struct rusage *t);

vector<double> toDoubles(const std::string &s);

int getAngleShiftFFT(const vector<double> &fft1, const vector<double> &ftd2);

} // namespace Laloc
} // namespace lama

#endif // _NJ_LASER_LALOC_UTILS_H_

