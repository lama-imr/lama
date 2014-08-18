
#ifndef _NJ_LASER_LALOC_UTILS_H_
#define _NJ_LASER_LALOC_UTILS_H_

#include <vector>
#include <string>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>

namespace Lama {
namespace Laloc {

using std::vector;

struct SPoint
{
  double x;
  double y;
  SPoint() : x(0), y(0) {}
  SPoint(const double xx, const double yy) : x(xx), y(yy) {}
  SPoint(const SPoint &p) : x(p.x), y(p.y) {}

  bool operator==(const SPoint &p)
  {
    return (p.x == x && p.y == y);
  }
};

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

    Segment(const SPoint &p1, const int i1,
        const SPoint &p2, const int i2) :
      x1(p1.x), y1(p1.y), from(i1), x2(p2.x), y2(p2.y), to(i2)
    {
    }

    double x1,y1;	
    int from;
    double x2;
    double y2;
    int to;
};

void setRandom();

std::vector<SPoint> cutScan(const std::vector<double> &scan,
    const double maxPhi, const double rt);

/* save scan to a file
 * maxPhi is in degrees
 */
void saveScan(const char *name, const std::vector<double> &range, const double maxPhi=360.0);

void saveScan(const char *filename, const std::vector<SPoint> &pts);

void saveDoubles(const char *filename, const std::vector<double> &values);

double getTime(struct rusage one, struct rusage two);
void getTime(struct rusage *t);

std::vector<double> toDoubles(const std::string &s);

int getAngleShiftFFT(const vector<double> &fft1, const vector<double> &ftd2);

} // namespace Laloc
} // namespace Lama

#endif // _NJ_LASER_LALOC_UTILS_H_

