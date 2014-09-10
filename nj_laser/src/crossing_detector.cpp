#include <nj_laser/crossing_detector.h>

namespace lama {
namespace nj_laser {

using std::list;
using std::vector;
using std::cerr;
using std::pair;
using std::stringstream;

const double CrossingDetector::frontier_width = 0.7;
const int CrossingDetector::descriptorFFTSize = 50;

const char* similarityMethodName[] = {
  "NCC_FFT",
  "POLYGON_BASED_SIMILARITY",
  "NCC_RANGE",
  "SHAPE_SIM",
  "NCC_FFT_FROM_SCAN"
};


/** notes:
  * actDescriptor contains only RANGES_FROM_RANGE_FINDER
  * but ! received vertex contain this descriptor plus another data, such as
  * resolution of range finder or number of angles per scan
  * so: 
  * working with actDescriptor .. it is ok
  * work with vertex data: descriptor must be extracted by  getDataFromVertex
  * it means: actDescriptor =def= getDataFromVertex(vertexData)
  */

/** vertex format:
  * vertex is array of numbers x[], where
  * x[0] .. radius of largest free circle in polygon
  * x[1] .. angle of range finder beam (eg. 2*PI or PI)
  * x[2] .. min angle of range finder beam, (eg. 0 for sick, or -40*PI/180 for hockuyo)
  * x[3] .. angle resolution (eg. 0.25 for sick)
  * x[4..n] .. data from range finder in metres
  *
  * all angles are in radian
  * TODO: clarify if angle resolution is in degree or radian
  * all distances are in metres
  */ 


CrossingDetector::CrossingDetector()
{
  //	similarityType = NCC_FFT;
  similarityType = NCC_FFT_FROM_SCAN;
  //maxScanPhi = 3/2*M_PI;
  maxScanPhi = 2 * M_PI;

  beamAngleSize = M_PI;
  beamOffset = 0;
  //beamResolution =0.00436;//0.25*M_PI/180.0;
  beamResolution = 0.25 * M_PI / 180.0;

  ROS_INFO("CrossingDetector: chosen similarity method is %s", similarityMethodName[similarityType]);
  ROS_INFO("CrossingDetector: descriptor size = %i", descriptorFFTSize);
  ROS_INFO("CrossingDetector: distance threshold frontier_width=%f", frontier_width);

  ROS_INFO("CrossingDetector: beamAngleSize = %f", beamAngleSize);
  ROS_INFO("CrossingDetector: beamOffset = %f", beamOffset);
  ROS_INFO("CrossingDetector: beamResolution = %f", beamResolution);
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
  for (auto range : scan.ranges)
    actDescriptor.push_back(range);
}




const vector<double>& CrossingDetector::getDescriptor() const
{
  return actDescriptor;
}

const vector<double>& CrossingDetector::getCrossDescriptor() const
{
  return actCrossdescriptor;
}




/*
 * @return data part of a vertex 
 * see comment above for description of data in vertex
 */
vector<double> CrossingDetector::getDataFromVertex(const vector<double> &vertex) const
{
  if (vertex.size() < 5)
  {
    return vector<double>();
  }
  return vector<double>(vertex.begin() + 4, vertex.end());
}

/* Compute and return the crossing centers
 *
 * scan list of laser ranges
 * minPhi angle of the first laser point in radian
 * maxPhi angle of the last laser point in radian
 */
void CrossingDetector::crossDetect(const sensor_msgs::LaserScan& scan)
{ 
  const double minPhi = scan.angle_min;
  const double maxPhi = scan.angle_max;
  std::vector<double> ranges(scan.ranges.size());
  for (auto range : scan.ranges)
    ranges.push_back(range);

  ros::Time t1;
  ros::Time t2;
  ros::Duration dt;


  double maxRange = *std::max_element(ranges.begin(), ranges.end());
  double rtOpt = 0.9 * maxRange;

  vector<Frontier> frontiers;
  t1 = ros::Time::now();
  cdPanoramatic3(ranges, rtOpt, frontier_width, minPhi, maxPhi, 45 * M_PI / 180.0, frontiers);
  t2 = ros::Time::now();
  dt = t2 - t1;
  ROS_DEBUG("Time of detecting exits from cross: %.1f s", dt.toSec());

  vector<double> res;

  if (frontiers.size() == 0)
  {
    res.push_back(0);
    res.push_back(0);
    res.push_back(-1);
    double m = -1;
    int mi = 0;
    for(int k = 0; k < ranges.size(); k++)
    {
      if (ranges[k] > m || m == -1)
      {
        mi = k;
        m = ranges[k];
      }
    }
    res.push_back(mi*maxScanPhi/ranges.size());
  }
  else
  {
    double cx,cy,cr;
    t1 = ros::Time::now();
    //getCrossCenterVoronoi(cutScan(ranges,maxScanPhi,rtOpt),rtOpt,frontier_width,cx,cy,cr);
    getCrossCenterVoronoiWithKDTree(cutScan(ranges,maxScanPhi,rtOpt),rtOpt,frontier_width,cx,cy,cr);
    t2 = ros::Time::now();
    dt = t2 - t1;
    ROS_DEBUG("Time of cross center search: %.1f s", dt.toSec());
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

int CrossingDetector::getDescriptorFFTSize() const
{
  return descriptorFFTSize;
}

/**
 * return string with answer for GET_VERTEX 
 *
 * see comment in begging of this file with description of data format
 */

std::string CrossingDetector::getLocalizeMessage(const double xpos, const double ypos) const
{
  /*
     char tmp[200];

     sprintf(tmp,"<VERTEX x=\"%lf\" y=\"%lf\">",xpos,ypos);

     std::string tmps = tmp;

     sprintf(tmp,"%lf ",getCrossRadius());
     tmps+=tmp;

     sprintf(tmp,"%lf ",getBeamAngleSize());
     tmps+=tmp;

     sprintf(tmp,"%lf ",getBeamOffset());
     tmps+=tmp;

     sprintf(tmp,"%lf ",getBeamResolution());
     tmps+=tmp;

     for(int i=0;i<actDescriptor.size();i++) {
     sprintf(tmp,"%lf ",actDescriptor[i]);
     tmps+=tmp;
     }	
     tmps+="</VERTEX>";
     return tmps;
     */
  stringstream ss;
  ss << "<VERTEX x=\"" << xpos << "\" y=\"" << ypos << "\">" << getCrossRadius() << " " 
    << getBeamAngleSize() << " " << getBeamOffset() << " " << getBeamResolution() << " ";
  for(int i=0;i<actDescriptor.size();i++) {
    ss << actDescriptor[i] << " ";
  }
  ss << "</VERTEX>";
  return ss.str();
}

std::string CrossingDetector::getInitMessage() const
{
  stringstream ss;
  ss <<  "<LOCALIZING>\n";
  ss << "<ALGORITHM type=\"LALOC\" version=\"1.0\" anytime=\"TRUE\">";
  ss << "<DEVICE type=\"LASER\" access=\"READ\"/>";
  ss << "</ALGORITHM>";
  return ss.str();
}

std::string CrossingDetector::getFinishMessage() const
{
  std::string tmp = "</LOCALIZING>";
  return tmp;
}

std::string CrossingDetector::getLocalizedMessage(const double prob, const double shift ) const
{
  stringstream ss;

  ss << "<POSITION probability=\"" << prob << "\">"<<shift<<"</POSITION>";
  /*
     char tmps[200];
     std::string tmp = "<POSITION probability=\"";
     sprintf(tmps,"%lf",prob);
     tmp+=tmps;
     tmp+="\"> ";
     sprintf(tmps,"%lf ",shift);
     tmp+=tmps;
     tmp+="</POSITION>";
     return tmp;
     */
  return ss.str();

}

/** for each detected edge (exit from a cross) send 
 * one edge */
std::string CrossingDetector::getEdgesMessage() const
{
  stringstream ss;

  ss << "<EDGES>";
  for(int i=0;i<actFrontiers.size();i++) {
    ss << "<EDGE width=\"" << actFrontiers[i].width << "\">" << actFrontiers[i].angle << "</EDGE>";
  }
  ss << "<EDGES/>";

  return ss.str();

}

void CrossingDetector::setMaxPhi(const double phi)
{
  maxScanPhi = phi;
}	

double CrossingDetector::getMaxPhi(const double phi) const
{
  return maxScanPhi;
}

double CrossingDetector::getBeamAngleSize() const
{
  return beamAngleSize;
}

double CrossingDetector::getBeamOffset() const
{
  return beamOffset;
}

double CrossingDetector::getBeamResolution() const
{
  return beamResolution;
}

void CrossingDetector::setBeamAngleSize(const double size)
{
  beamAngleSize = size;
  ROS_INFO("CrossingDetector: beamAngleSize set to %f" ,size);
}


void CrossingDetector::setBeamOffset(const double offset)
{
  beamOffset = offset;
  ROS_INFO("CrossingDetector: beamOffset set to %f" ,offset);
}

void CrossingDetector::setBeamResolution(const double resolution)
{
  beamResolution = resolution;
  ROS_INFO("CrossingDetector: beamResolution set to %f " , resolution);
}

int CrossingDetector::getDescriptorSize() const
{
  return actDescriptor.size();
}

} // namespace nj_laser
} // namespace lama


