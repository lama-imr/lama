
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <nj_laser/laloc_utils.h>

namespace lama {
namespace nj_laser {

using std::cerr;
using std::vector;
using std::ofstream;
using std::string;

/* Make a list of point from current scan. Points farther than threshold rt are skipped.
*/
vector<Point2> cutScan(const vector<double> &scan,
    const double maxPhi, const double rt)
{
  vector<Point2> p;

  const int sSize = scan.size();

  Point2 tmp;
  for(int i = 0; i < sSize; i++)
  {
    if (scan[i] <= rt)
    {
      tmp.x = scan[i] * cos(i * maxPhi / sSize);
      tmp.y = scan[i] * sin(i * maxPhi / sSize);
      p.push_back(tmp);
    }
  }
  return p;
}

/** save scan to gnuplot
*/
void saveScan(const char *name, const vector<double> &range, const double maxPhi)
{
  ofstream ofs;
  ofs.open(name);

  const int rSize = range.size();
  double x,y;
  for(int i=0;i<rSize;i++) {
    x = range[i]*cos(i*maxPhi/rSize);
    y = range[i]*sin(i*maxPhi/rSize);
    ofs << x<<" "<<y<<" "<<i*maxPhi/rSize <<" "<<range[i]<<"\n";
  }
  ofs.close();
}


void saveScan(const char *filename, const vector<Point2> &pts) 
{
  ofstream ofs(filename);

  for(int i = 0; i < pts.size(); i++)
    ofs << pts[i].x << " " << pts[i].y << "\n";
  ofs.close();
}

void saveDoubles(const char *filename, const std::vector<double> &values)
{
  ofstream ofs(filename);

  for(int i = 0; i < values.size(); i++)
  {
    ofs << values[i] << "\n";
  }
  ofs.close();
}

} // namespace nj_laser
} // namespace lama

