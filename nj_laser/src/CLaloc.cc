
#include "CLaloc.h"
#include "CHist.h"
#include "crossDetect.h"
#include "ros/ros.h"


#include <math.h>
#include <algorithm>
#include <list>
#include <sstream>

namespace Lama {
namespace Laloc {

using std::list;
using std::vector;
using std::cerr;
using std::pair;
using std::stringstream;

const double CLaloc::dt = 0.7;
const int CLaloc::descriptorFFTSize = 50;

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
  * it means: actDesctiptor =def= getDataFromVertex(vertexData)
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


CLaloc::CLaloc()
{
	//	similarityType = NCC_FFT;
	similarityType = NCC_FFT_FROM_SCAN;
	//maxScanPhi = 3/2*M_PI;
	maxScanPhi = 2 * M_PI;

	beamAngleSize = M_PI;
	beamOffset = 0;
	//beamResolution =0.00436;//0.25*M_PI/180.0;
	beamResolution = 0.25 * M_PI / 180.0;

	ROS_INFO("CLaloc: chosen similarity method is %s", similarityMethodName[similarityType]);
	ROS_INFO("CLaloc: descriptor size = %i", descriptorFFTSize);
	ROS_INFO("CLaloc: distance threshold dt=%f", dt);

	ROS_INFO("CLaloc: beamAngleSize = %f", beamAngleSize);
	ROS_INFO("CLaloc: beamOffset = %f", beamOffset);
	ROS_INFO("CLaloc: beamResolution = %f", beamResolution);
}

CLaloc::~CLaloc()
{
	actDescriptor.clear();
	actCrossdescriptor.clear();
}

void CLaloc::setDescriptor(const vector<double> &scan, const double minPhi, const double maxPhi) {
	crossDetect(scan, minPhi, maxPhi);
	actDescriptor = scan;
}




const vector<double> &CLaloc::getDescriptor() const {
	return actDescriptor;
}

const vector<double> &CLaloc::getCrossDescriptor() const {
	return actCrossdescriptor;
}




/*
 * @return data part of a vertex 
 * see comment above for description of data in vertex
 */
vector<double> CLaloc::getDataFromVertex(const vector<double> &vertex) const {
	if (vertex.size() < 5) {
		return vector<double>();
	}
	return vector<double>(vertex.begin()+4,vertex.end());
}

/* Compute and return the crossing centers
 *
 * scan list of laser ranges
 * minPhi angle of the first laser point in radian
 * maxPhi angle of the last laser point in radian
 */
void CLaloc::crossDetect(const vector<double> &scan, const double minPhi, const double maxPhi)
{
	struct rusage t1,t2;

	
	double maxRange = *std::max_element(scan.begin(), scan.end());
	//getTime(&t1);
	//CHist hist(0, maxRange, maxRange / 30);
	//hist.add(scan);

	//std::vector<int> v = hist.getHist();
	//std::ostringstream s;
	//s << "hist:";
	//for(std::vector<int>::iterator it = v.begin(); it != v.end(); ++it)
	//{
	//	s << *it << ",";
	//}
	//ROS_INFO("%s", s.str().c_str());

	// rtOpt is the range value where the most range values are. In mostly free
	// space, this corresponds to the max. laser range.
	//double rtOpt = 0.95 * hist.getRangeLo(hist.getMaxBin());

	//getTime(&t2);
	//ROS_INFO("time of making histogram: %f s, rtOpt: %f, maxrange: %f", getTime(t1,t2), rtOpt, maxRange);
	double rtOpt = 0.9 * maxRange;

	vector<SFrontier> frontiers;
	getTime(&t1);
	cdPanoramatic3(scan, rtOpt, dt, minPhi, maxPhi, 45 * M_PI / 180.0, frontiers);
	getTime(&t2);
	//	ROS_INFO("time of detecting exits from cross: " << getTime(t1,t2)<<" s");

	vector<double> res;

	if (frontiers.size() == 0) {
		res.push_back(0);
		res.push_back(0);
		res.push_back(-1);
		double m = -1;
		int mi = 0;
		for(int k = 0; k<scan.size();k++)
			if (scan[k] > m || m == -1) {
				mi = k;
				m = scan[k];
			}
		res.push_back(mi*maxScanPhi/scan.size());
	} else {
		double cx,cy,cr;
		getTime(&t1);
		//getCrossCenterVoronoi(cutScan(scan,maxScanPhi,rtOpt),rtOpt,dt,cx,cy,cr);
		getCrossCenterVoronoiWithKDTree(cutScan(scan,maxScanPhi,rtOpt),rtOpt,dt,cx,cy,cr);
		getTime(&t2);
		//ROS_INFO("time of cross center search: " << getTime(t1,t2) << " s");
		res.push_back(-cy); // TODO: explain why -cy (minus sign and y before x)
		res.push_back(-cx);
		res.push_back(cr);
		for(int i=0;i<frontiers.size();i++) {
			res.push_back(frontiers[i].angle);
		}
	}
	actFrontiers = frontiers;
	frontiers.clear();
	actCrossdescriptor = res;
}

double CLaloc::getCrossCenterX() const {
	if (actCrossdescriptor.size() > 0)
		return actCrossdescriptor[0];
	return 0;
}

double CLaloc::getCrossCenterY() const {
	if (actCrossdescriptor.size() > 1)
		return actCrossdescriptor[1];
	return 0;
}

double CLaloc::getCrossRadius() const {
	if (actCrossdescriptor.size() > 2)
		return actCrossdescriptor[2];
	return 0;
}

int CLaloc::getNumExits() const {
	return std::max(0,(int)(actCrossdescriptor.size() - 3));
}

double CLaloc::getExitAngle(const int i) const {
//	if (getNumExits() > 0 && i<getNumExits()) {
//		return actCrossdescriptor[3+i];
//	} else
//		return 0;
	if (i >=0 && i < actFrontiers.size()) {
		return actFrontiers[i].angle;
	}
	return 0;
}

double CLaloc::getExitWidth(const int i) const {
	if (i >= 0 && i < actFrontiers.size()) {
		return actFrontiers[i].width;
	}
	return -1;
}

int CLaloc::getDescriptorFFTSize() const {
	return descriptorFFTSize;
}

/**
  * return string with answer for GET_VERTEX 
  *
  * see comment in begging of this file with description of data format
  */
  
std::string CLaloc::getLocalizeMessage(const double xpos, const double ypos) const {
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

std::string CLaloc::getInitMessage() const {
	stringstream ss;
	ss <<  "<LOCALIZING>\n";
	ss << "<ALGORITHM type=\"LALOC\" version=\"1.0\" anytime=\"TRUE\">";
	ss << "<DEVICE type=\"LASER\" access=\"READ\"/>";
	ss << "</ALGORITHM>";
	return ss.str();
}

std::string CLaloc::getFinishMessage() const {
	std::string tmp = "</LOCALIZING>";
	return tmp;
}

std::string CLaloc::getLocalizedMessage(const double prob, const double shift ) const {
	
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
std::string CLaloc::getEdgesMessage() const {

	stringstream ss;

	ss << "<EDGES>";
	for(int i=0;i<actFrontiers.size();i++) {
		ss << "<EDGE width=\"" << actFrontiers[i].width << "\">" << actFrontiers[i].angle << "</EDGE>";
	}
	ss << "<EDGES/>";

	return ss.str();

}
void CLaloc::setMaxPhi(const double phi) {
	maxScanPhi = phi;
}	


double CLaloc::getMaxPhi(const double phi) const {
	return maxScanPhi;
}

double CLaloc::getBeamAngleSize() const {
	return beamAngleSize;
}

double CLaloc::getBeamOffset() const {
	return beamOffset;
}

double CLaloc::getBeamResolution() const {
	return beamResolution;
}

void CLaloc::setBeamAngleSize(const double size) {
	beamAngleSize = size;
	ROS_INFO("CLaloc: beamAngleSize set to %f" ,size);
}


void CLaloc::setBeamOffset(const double offset) {
	beamOffset = offset;
	ROS_INFO("CLaloc: beamOffset set to %f" ,offset);
}

void CLaloc::setBeamResolution(const double resolution) {
	beamResolution = resolution;
	ROS_INFO("CLaloc: beamResolution set to %f " , resolution);
}

int CLaloc::getDescriptorSize() const {
	return actDescriptor.size();
}

} // namespace Laloc
} // namespace Lama


