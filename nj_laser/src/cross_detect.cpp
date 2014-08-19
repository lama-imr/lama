
#include <iostream>
#include <math.h>
#include <fstream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nj_laser/cross_detect.h>
#include <nj_laser/laloc_utils.h>
#include <nj_laser/voronoi.h>
#include <nj_laser/polygon_utils.h>
#include <nj_laser/shape_sim.h>
#include <ANN/ANN.h>

/*
 * functions for detecting exit from a cross using data from
 * laser range finder
 */

namespace lama {
namespace Laloc {

using std::vector;
using lama::Point2;

// A map to store previously computed distances with ANN
// first 16 bit = x * 100, last 16 bit = y * 100
typedef std::map<uint32_t, double> freeSpace_dict;

	
struct myLessPts {
	bool operator()(const Point2 &a, const Point2 &b) {
		if (a.x < b.x) {
			return true;
		}

		if (a.x > b.x) {
			return false;
		}

		if (a.y < b.y) {
			return true;
		}

		return false;
	}
};


struct VDEdge {
	double x1,y1,x2,y2;
	int i1, i2;
	VDEdge(const double _x1, const double _y1, const double _x2, const double _y2,
			const int _i1=0, const int _i2=0):
		x1(_x1),y1(_y1),x2(_x2),y2(_y2),i1(_i1),i2(_i2) {}

};

struct CenterC {
	double x1,y1;
	double r;
	CenterC(const double _x1, const double _y1, const double _r):
		x1(_x1),y1(_y1),r(_r) {}
};

struct SCircle {
	double x,y,r;
	SCircle(const double xx, const double yy, const double rr):
		x(xx),y(yy),r(rr) {}
};	

/* return frontiers and angles to it
 *
 * scan laser ranges
 * rt max range for a laser beam to be considered
 * dt min frontier width
 * minPhi angle of first laser point in radians.
 * maxPhi angle of last laser point in radians
 * maxFrontierAngle max. allowed frontier angle in radians (0 means angle
 *                  between line from robot to frontier middle and frontier is
 *                  90 deg).
 * frontiers returned frontiers
 */
void cdPanoramatic3(
	const vector<double> &scan, const double rt, const double dt,
	const double minPhi, const double maxPhi, const double maxFrontierAngle,
	vector<SFrontier> &frontiers)  {

    vector<double> filtScan;
    vector<int> angleNumber;

	// Filter out laser beams longer than rt
    filtScan.reserve(scan.size());
    angleNumber.reserve(scan.size());
	for(int i = 0; i < scan.size(); i++) {
		if (scan[i] < rt) {
			filtScan.push_back(scan[i]);
            angleNumber.push_back(i);
        }
    }


    if (filtScan.size() == 0) {
		std::cerr << __FUNCTION__ << " all points from scan are above threshold\n";
        return;
    }

	if (scan.size() < 2)
	{
		std::cerr << __FUNCTION__ << " laser scan must have at least 2 points\n";
		return;
	}

	double phiResolution = (maxPhi - minPhi) / (scan.size() - 1);
	double aAngle, bAngle;
    Point2 a, b;
	aAngle = angleNumber[0] * phiResolution; //TODO: was aAngle = angleNumber[0];, explain
    a.x = filtScan[0] * cos(minPhi + aAngle);
    a.y = filtScan[0] * sin(minPhi + aAngle);
	 
    double dist;
	frontiers.clear();

    for(int i = 1; i < filtScan.size() + 1; i++)
	{
		int j = i % filtScan.size();
		bAngle = angleNumber[j] * phiResolution; //TODO: was bAngle = angleNumber[j];, explain
        b.x = filtScan[j] * cos(minPhi + bAngle);
        b.y = filtScan[j] * sin(minPhi + bAngle);
        dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
		  
        if (dist > dt*dt)
		{
            double sx = (a.x + b.x) / 2.0;
            double sy = (a.y + b.y) / 2.0;

			double distToFrontierCenter = sqrt(sx * sx + sy * sy);
			double dotProductFrontierSxSy = (b.x - a.x) * sx + (b.y - a.y) * sy;
			double frontierAngleWithSxSy = acos(dotProductFrontierSxSy / dist / distToFrontierCenter);
			if (fabs(M_PI_2 - frontierAngleWithSxSy) < maxFrontierAngle) //TODO: was fabs(aAngle-bAngle) > maxFrontierAngle)
			{
				frontiers.push_back(SFrontier(a, b, sqrt(dist), atan2(sy,sx)));
			}
        }
        a.x = b.x;
        a.y = b.y;
		aAngle = bAngle;
	}

	// TODO: frontiers[i].angle is already normalized with ]-pi,pi], explain why [0, 2*pi[ needed.
	for(int i=0;i<frontiers.size();i++) {
		while(frontiers[i].angle < 0) {
			frontiers[i].angle += 2*M_PI;
		}
	}
}

/*
 * return the barycenter of a list of points
 */
void getCenter(const vector<Point2> &pts, double &cx, double &cy) {
	double sx = 0;
	double sy = 0;

	for(int i=0;i<pts.size();i++) {
		sx+=pts[i].x;
		sy+=pts[i].y;
	}
	cx=sx/pts.size();
	cy=sy/pts.size();
}


/** return center of polygon. 
  */
void getPolygonCenter(const vector<Point2> &p,
		double &cx, double &cy) {

	cx = 0;
	cy = 0;


	const size_t s = p.size();
	/* vypocet plochy polygonu */
	double a = 0;
	for(size_t i=0;i<s;i++)
		a+=p[i%s].x * p[(i+1)%s].y - 
		   p[(i+1)%s].x * p[i%s].y;

	a /=2.0;
	a = fabs(a);

	/* vypocet stredu polygonu */
	for(size_t i=0;i<s;i++) {
		cx+=(p[i].x+p[(i+1)%s].x)*
			(p[i].x * p[(i+1)%s].y - p[(i+1)%s].x * p[i].y);
		
		cy+=(p[i].y+p[(i+1)%s].y)*
			(p[i].x * p[(i+1)%s].y - p[(i+1)%s].x * p[i].y);
	}

	cx/=(6*a);
	cy/=(6*a);

}

/** return center of a crossing
 * angles Angles at which range measurements are done in radians
 * scan Measured ranges
 * maxPhi Field of view of range measurements in radians
 * corners Returned 
 */
void getCrossCenter(const vector<double> &angles,
		const vector<double> &scan, const double maxPhi,
		vector<Point2> &corners, double &cx, double &cy)
{


	cx = 0;
	cy = 0;
	corners.clear();
	
	int n = scan.size();
	int j;
	Point2 pt;
	for(int i = 0; i < angles.size(); i++) {
		j = (i + 1) % angles.size();
		int from = (int)floor(angles[i] * n / maxPhi);
		int to = (int)ceil(angles[j] * n / maxPhi);

		int localMinIndex = from;
		if (from <= to) {
			for(int k = from; k <= to; k++) {
				if (scan[k % n] < scan[localMinIndex])
					localMinIndex = k % n;
			}
		} else {
			for(int k = from; k < n; k++)
				if (scan[k % n] < scan[localMinIndex])
					localMinIndex = k % n;
			for(int k=0; k < to; k++)
				if (scan[k % n] < scan[localMinIndex])
					localMinIndex = k % n;
		}
		
		pt.x = scan[localMinIndex]*cos(localMinIndex*maxPhi/scan.size());
		pt.y = scan[localMinIndex]*sin(localMinIndex*maxPhi/scan.size());
		corners.push_back(pt);
	}

	// in pts are point at corners
	getCenter(corners,cx,cy);

}

/** replace all lines by some points if length of that lines is
  * longer than dt
  */
template<typename PT>
vector<PT> replaceFrontiersT(const vector<Point2> &pts, const double dt) {

	const int size = pts.size();
	const double dt2 = dt*dt;
	double s;

	vector<PT> tmp;
	tmp.reserve(size);

	for(int i=0;i<size;i++) {
		int j = (i+1) % size;

		double dx = pts[i].x - pts[j].x;
		double dy = pts[i].y - pts[j].y;

		if (dx*dx + dy*dy> dt2) {

			s = sqrt(dx*dx+dy*dy) / (0.2*dt);
			for(double k=0;k<=1;k+=(1/s)){
				tmp.push_back(PT((1-k)*pts[i].x+k*pts[j].x,
							         (1-k)*pts[i].y+k*pts[j].y));
			}

		} else
			tmp.push_back(PT(pts[i].x,pts[i].y));

	}
	return tmp;
}



/** replace all lines by some points if length of that lines is
  * longer than dt
  */
vector<Point2> replaceFrontiers(const vector<Point2> &pts, const double dt) {

	const int size = pts.size();
	const double dt2 = dt*dt;
	double s;

	vector<Point2> tmp;
	tmp.reserve(size);

	for(int i=0;i<size;i++) {
		int j = (i+1) % size;

		double dx = pts[i].x - pts[j].x;
		double dy = pts[i].y - pts[j].y;

		if (dx*dx + dy*dy> dt2) {

			s = sqrt(dx*dx+dy*dy) / (0.2*dt);
			for(double k=0;k<=1;k+=(1/s)){
				tmp.push_back(Point2((1-k)*pts[i].x+k*pts[j].x,
							         (1-k)*pts[i].y+k*pts[j].y));
			}

		} else
			tmp.push_back(pts[i]);

	}
	return tmp;

}


int getIndex(const vector<double> &array, const double a) {

	int i=0;
	while(i<array.size() && array[i] < a) i++;

	return i;	
}

/**
  * remove all edges that are behind scan points
  */
template<typename PT>
void filterVoronoiEdges(vector<VDEdge> &edges, const vector<PT> &pts) {

	vector<VDEdge> newEdges;

	vector<double> angles;
	angles.reserve(pts.size());

	double d,dx;
	for(int i=0;i<pts.size();i++) {
		d = atan2(pts[i].y,pts[i].x);
		if (d<0) d+=2*M_PI;
		angles.push_back(d);
	}

	bool far;
	int from,to;

	const int pSize = pts.size();
	const int eSize = edges.size();

	
	for(int u=0;u<eSize;u++) {
	
			const double x1 = edges[u].x1;
			const double y1 = edges[u].y1;
			const double x2 = edges[u].x2;
			const double y2 = edges[u].y2;

			// if some point in edge is far then actual scan in given
			// angle, do not put such edge to vector
			d = atan2(y1,x1);
			if (d<0) d+=2*M_PI;
			
			dx = x1*x1+y1*y1;
			
			
			far = false;
			int f = getIndex(angles,d);
			if (f < angles.size()) {
				if (pts[f].x*pts[f].x + pts[f].y*pts[f].y < dx)
					far = true;
			}	
			if ((f-1)%angles.size() > 0) {
				int j = (f-1) % angles.size();
				if (pts[j].x*pts[j].x + pts[j].y*pts[j].y < dx)
					far = true;
			}

			if (far == false) {
				d = atan2(y2,x2);
				if (d<0) d+=2*M_PI;
				dx = x2*x2+y2*y2;
	
				f = getIndex(angles,d);
				if (f < angles.size()) {
					if (pts[f].x*pts[f].x + pts[f].y*pts[f].y < dx)
						far = true;
				}	
				if ((f-1)%angles.size() > 0) {
					int j = (f-1) % angles.size();
					if (pts[j].x*pts[j].x + pts[j].y*pts[j].y < dx)
						far = true;
				}
			}



			if (!far) 
				newEdges.push_back(VDEdge(x1,y1,x2,y2));
	}
	edges.clear();
	angles.clear();
	edges = newEdges;		

}

// return min/max values of points
// return true, if boundary values are valid
template<typename PT>
bool getMinMaxValues(const vector<PT> &pts, double &minx, double &maxx,
		double &miny, double &maxy) {

	const int size = pts.size();

	if (size == 0) {
		return false;
	}

	minx = pts[0].x;
	maxx = minx;
	miny = pts[0].y;
	maxy = miny;

	int i=0;
	if (size & 1 == 1) {
		i = 1;
	}

	for(;i<size;i+=2) {
		if (pts[i].x > pts[i+1].x) {
			if (pts[i].x > maxx) {
				maxx = pts[i].x;
			}
			if (pts[i+1].x < minx) {
				minx = pts[i+1].x;
			}
		} else {
			if (pts[i+1].x > maxx) {
				maxx = pts[i+1].x;
			}
			if (pts[i].x < minx) {
				minx = pts[i].x;
			}

		}
		if (pts[i].y > pts[i+1].y) {
			if (pts[i].y > maxy) {
				maxy = pts[i].y;
			}
			if (pts[i+1].y < miny) {
				miny = pts[i+1].y;
			}
		} else {
			if (pts[i+1].y > maxy) {
				maxy = pts[i+1].y;
			}
			if (pts[i].y < miny) {
				miny = pts[i].y;
			}

		}

	}
	return true;

}

/* Return the integer representing a point
 *
 * The first 16 bit are a signed integer equal to the x-coordinate multiplied
 * by 100. The last 16 bit are for y.
 */
void encodePoint(const double x, const double y, uint32_t& code)
{
	if (((x * 100) < (((double) INT16_MIN) + 1)) ||
			((x * 100) > (((double) INT16_MAX) - 1)) ||
			((y * 100) < (((double) INT16_MIN) + 1)) ||
			((y * 100) > (((double) INT16_MAX) - 1)))
	{
		ROS_ERROR("Point (%f, %f) cannot be encoded", x, y);
	}
	code = 0;
	int16_t xint = (int16_t) round(x * 100);
	int16_t yint = (int16_t) round(y * 100);
	code |= (xint << 16);
	code |= (yint & 0x0000FFFF);
}

/* Return the point represented by an integer
 *
 * The first 16 bit of the coded integer are a signed integer equal to the
 * x-coordinate multiplied by 100. The last 16 bit are for y.
 */
void decodePoint(const uint32_t code, double& x, double& y)
{
	int16_t xint = (int16_t) (code >> 16);
	int16_t yint = (int16_t) code;
	x = ((double) xint) / 100;
	y = ((double) yint) / 100;
}

// Return the largest circle (x, y, radius^2) of free space around each vertex.
//
// edges[in] edges, the vertices of which are the circle centers
// pts[in] pts representing obstacles
vector<CenterC> getFreeSpace(const vector<VDEdge> edges, const vector<Voronoi::Point> pts)
{
	// build KD-tree from the pts array 
	const int k = 1; // Max. number of nearest neighbors
	ANNidxArray idx = new ANNidx[k];
	ANNdistArray dist = new ANNdist[k];
	ANNpoint query = annAllocPt(2);	

	ANNpointArray ap = annAllocPts(pts.size(), 2);
	for(unsigned i = 0; i < pts.size(); ++i)
	{
		ap[i][0] = pts[i].x;
		ap[i][1] = pts[i].y;
	}

	ANNkd_tree* tree = new ANNkd_tree(ap, pts.size(), 2);

	// freeSpace_d is a map used to saved already computed nearest neighbors,
	// allowing to improve the comparison by computing each node only once.
	freeSpace_dict freeSpace_d;
	unsigned int ann_count = 0;
	for(auto edge : edges)
	{
		uint32_t codedPoint;
		encodePoint(edge.x1, edge.y1, codedPoint);
		freeSpace_dict::const_iterator search = freeSpace_d.find(codedPoint);
		if (search == freeSpace_d.end())
		{
			// Point was not yet calculated.
			query[0] = edge.x1;
			query[1] = edge.y1;
			// TODO: test eps value (last parameter of annkSearch)
			// TODO: try tree->annkFRSearch (same as annkSearch with max. radius)
			tree->annkSearch(query, k, idx, dist, 0.01);
			ann_count++;
			if (dist[0] > 0)
			{
				freeSpace_d[codedPoint] = dist[0];
			}
		}
		encodePoint(edge.x2, edge.y2, codedPoint);
		search = freeSpace_d.find(codedPoint);
		if (search == freeSpace_d.end())
		{
			query[0] = edge.x2;
			query[1] = edge.y2;
			tree->annkSearch(query, k, idx, dist, 0);
			ann_count++;
			if (dist[0] > 0)
			{
				freeSpace_d[codedPoint] = dist[0];
			}
		}
	}
	ROS_DEBUG("Number of nearest neighbor searches %u", ann_count);

	vector<CenterC> freeSpace;
	freeSpace.reserve(freeSpace_d.size());
	for (auto pair : freeSpace_d)
	{
		double x, y;
		decodePoint(pair.first, x, y);
		freeSpace.push_back(CenterC(x, y, pair.second));
	}
	ROS_DEBUG("Number of crossing center candidates: %lu", freeSpace.size());

	delete [] idx;
	delete [] dist;
	delete tree;
	annDeallocPt(query);
	annDeallocPts(ap);
	annClose();

	return freeSpace;
}

/**
  * return cross center determined by voronoi diagram.
  * center is returned as triplet (cx,cy,r) where cx,cy uis center of circle with radius
  * 'r'. if no center is detected, triplet (0,0,-1) is returned.
  */
void getCrossCenterVoronoi(
		const vector<Point2> &pts,  const double rt, const double dt,
		double &cx, double &cy, double &radius) {

	if (pts.size() == 0) {
		cx = 0;
		cy = 0;
		radius = -1;
	}

	const double delta = 10;
	const double err = 0.01;

	// ROS_INFO("Filter: " << ptsRel.size() << " from " << pts.size());

	//	vector<Point2> mypts(replaceFrontiers(pts,dt));
	vector<Point2> mypts(replaceFrontiers(filterRelevance(pts,0.15),dt));
	
	double minx,maxx,miny,maxy;
	bool s = getMinMaxValues(mypts,minx,maxx,miny,maxy);

	vector<Voronoi::Point> vpts;
	for(int i=0;i<mypts.size();i++)
		vpts.push_back(Voronoi::Point(mypts[i].x,mypts[i].y));

	Voronoi::VoronoiDiagramGenerator *vd = new Voronoi::VoronoiDiagramGenerator();
	vd->generateVoronoi(&vpts,minx-delta,maxx+delta,miny-delta,maxy+delta,0);
	vd->resetIterator();
	vector<VDEdge> edges;

	float x1,y1,x2,y2;

 	Voronoi::GraphEdge eee;
	while(vd->getNext(eee)) {
		if ( !( eee.x1<minx || eee.y1<miny || eee.x1>maxx || eee.y1>maxy || 
			   eee.x2<minx || eee.y2<miny || eee.x2>maxx || eee.y2>maxy)) 
			edges.push_back(VDEdge(eee.x1,eee.y1,eee.x2,eee.y2));
	}
	eee.next = NULL;
	ROS_INFO("voronoi edges num = %lu" , edges.size());

	// remove edges behind scan points
	filterVoronoiEdges(edges,mypts);



	int mindj;
	double mind;
	double dx,dy,d;
	vector<CenterC> candidates;

	// make candidates for result.. 

	// bylo pts misto mypts
	for(int i=0;i<edges.size();i++) {
		mind = -1;
		for(int j=0;j<mypts.size();j++) {
			dx = edges[i].x1 - mypts[j].x;
			dy = edges[i].y1 - mypts[j].y;
			d = dx*dx+dy*dy;
			if (d<mind || mind == -1) {
				mind = d;
				mindj = j;
			}
		}
		candidates.push_back(CenterC(edges[i].x1,edges[i].y1,mind));

		mind = -1;
		for(int j=0;j<mypts.size();j++) {
			dx = edges[i].x2 - mypts[j].x;
			dy = edges[i].y2 - mypts[j].y;
			d = dx*dx+dy*dy;
			if (d<mind || mind == -1) {
				mind = d;
				mindj = j;
			}
		}

		candidates.push_back(CenterC(edges[i].x2,edges[i].y2,mind));

	}
	// find largest circle
	int maxr =0;
	for(int i=0;i<candidates.size();i++)
		if (candidates[i].r > candidates[maxr].r)
			maxr = i;

	if (candidates.size() > 0) {
		cx = candidates[maxr].x1;
		cy = candidates[maxr].y1;
		radius = sqrt(candidates[maxr].r);
	} else {
		cx = 0;
		cy = 0;
		radius = -1;
	}	

	candidates.clear();
	edges.clear();
	delete vd;
	vpts.clear();

}


/**
  * return cross center determined by voronoi diagram.
  * center is returned as triplet (cx,cy,r) where cx,cy uis center of circle with radius
  * 'r'. if no center is detected, triplet (0,0,-1) is returned.
  *
  * filtruje kandidaty na stred krizovatky dotazem point-in-polygon
  
  * postup: z bodu skenu (pts) se vyflitruji stejne body,
  * postavi se voronoi diagram
  * z neho se odstrani hrany lezici mimo polygon (tj. alespon jeden bod na hrane musi byt mimo polygon)
  * ze zbylych hran se urci nejvetsi volna kruznice lezici uvnitr polygonu
  */
void getCrossCenterVoronoiWithPointInPolygon(
		const vector<Point2> &pts, const vector<Point2> &frontiers,
		const double rt, const double dt, double &cx, double &cy, double &radius) {


	const double delta = 10;
	const double err = 0.01;

	// pts jsou v metrech
	std::vector<Point2> mypts = replaceFrontiers(pts,dt);

	vector<Point2> myPtsCopy(mypts);

	// remove unique points
	vector<Point2> ptsCopy(mypts.begin(), mypts.end());
	std::sort(ptsCopy.begin(),ptsCopy.end(),myLessPts());
	vector<Point2>::iterator newEnd = std::unique(ptsCopy.begin(),ptsCopy.end());

	mypts.clear();
	copy(ptsCopy.begin(),newEnd,std::back_inserter(mypts));
	ptsCopy.clear();

	double minx = mypts[0].x;
	double maxx = mypts[0].x;
	double miny = mypts[0].y;
	double maxy = mypts[0].y;

	for(int i=0;i<mypts.size();i++) {
		minx = std::min(minx,mypts[i].x);
		miny = std::min(miny,mypts[i].y);
		maxx = std::max(maxx,mypts[i].x);
		maxy = std::max(maxy,mypts[i].y);
	}

	std::vector<Voronoi::Point> vpts;
	for(int i=0;i<mypts.size();i++)
		vpts.push_back(Voronoi::Point(mypts[i].x,mypts[i].y));

	Voronoi::VoronoiDiagramGenerator *vd = new Voronoi::VoronoiDiagramGenerator();
	vd->generateVoronoi(&vpts,minx-delta,maxx+delta,miny-delta,maxy+delta,0);

	vd->resetIterator();
	std::vector<VDEdge> edges;

	int mmxp = 0;

 	Voronoi::GraphEdge eee;
	while(vd->getNext(eee)) {
		if ( !( eee.x1<minx || eee.y1<miny || eee.x1>maxx || eee.y1>maxy || 
			   eee.x2<minx || eee.y2<miny || eee.x2>maxx || eee.y2>maxy)) {

			edges.push_back(
					VDEdge(eee.x1,eee.y1,eee.x2,eee.y2,eee.point1,eee.point2));

		}
		mmxp = std::max(mmxp,std::max(eee.point1,eee.point2));
	}
	eee.next = NULL;

	std::vector<bool> used(mmxp,false);

	filterVoronoiEdges(edges,myPtsCopy);

	int mindj;
	double mind;
	double dx,dy,d;
	std::vector<CenterC> candidates;

	for(int i=0;i<edges.size();i++) {

		if (used[edges[i].i1] == false) {
			mind = -1;
			for(int j=0;j<mypts.size();j++) {
				dx = edges[i].x1 - mypts[j].x;
				dy = edges[i].y1 - mypts[j].y;
				d = dx*dx+dy*dy;
				if (d<mind || mind == -1) {
					mind = d;
					mindj = j;
				}
			}
			used[edges[i].i1] = true;
			candidates.push_back(CenterC(edges[i].x1,edges[i].y1,mind));
		}

		if (used[edges[i].i2] == false) {
			mind = -1;
			for(int j=0;j<mypts.size();j++) {
				dx = edges[i].x2 - mypts[j].x;
				dy = edges[i].y2 - mypts[j].y;
				d = dx*dx+dy*dy;
				if (d<mind || mind == -1) {
					mind = d;
					mindj = j;
				}
			}
			candidates.push_back(CenterC(edges[i].x2,edges[i].y2,mind));
			used[edges[i].i2] = true;
		}

	}
	
	// odstran body, ktere nejsou v polygonu

	typedef lama::PolygonUtils::TPoint<int> RPoint;
	vector<RPoint> polygon;
	polygon.reserve(pts.size()+1);
	for(int i=0;i<pts.size();i++) {
		polygon.push_back(RPoint((int)round(100*pts[i].x),(int)round(100*pts[i].y)));
	}
	if (polygon.size() > 0) {
		polygon.push_back(polygon.front());
	}


	int bestCandidate = -1;
	double maxRadius = -1;
	for(int i=0;i<candidates.size();i++) {
		if (wn_PnPoly(RPoint((int)round(100*candidates[i].x1),(int)round(100*candidates[i].y1)),polygon) != 0) {
			if (candidates[i].r > maxRadius || maxRadius == -1) {
				maxRadius = candidates[i].r;
				bestCandidate = i;
			}
		}
	}

	if (candidates.size() > 0) {
		cx = candidates[bestCandidate].x1;
		cy = candidates[bestCandidate].y1;
		radius = sqrt(candidates[bestCandidate].r);
	} else {
		cx = 0;
		cy = 0;
		radius = -1;
	}	

	candidates.clear();
	edges.clear();
	delete vd;
	vpts.clear();
	mypts.clear();
	used.clear();
	polygon.clear();
}



/**
  * return cross center determined by voronoi diagram.
  * center is returned as triplet (cx,cy,r) where cx,cy uis center of circle with radius
  * 'r'. if no center is detected, triplet (0,0,-1) is returned.
  */
void getCrossCenterVoronoiWithKDTree(
		const vector<Point2> &pts,  const double rt, const double dt,
		double &cx, double &cy, double &radius) {

	if (pts.size() == 0) {
		cx = 0;
		cy = 0;
		radius = -1;
	}

	const double delta = 10;
	const double err = 0.01;

	vector<Voronoi::Point> mypts(replaceFrontiersT<Voronoi::Point>(filterRelevance(pts, 0.15), dt));
	
	double minx,maxx,miny,maxy;
	bool s = getMinMaxValues(mypts,minx,maxx,miny,maxy);

	//	vector<Voronoi::Point> vpts;
	//	for(int i=0;i<mypts.size();i++)
	//		vpts.push_back(Voronoi::Point(mypts[i].x,mypts[i].y));


	Voronoi::VoronoiDiagramGenerator *vd = new Voronoi::VoronoiDiagramGenerator();
	vd->generateVoronoi(&mypts,minx-delta,maxx+delta,miny-delta,maxy+delta,0);
	vd->resetIterator();
	vector<VDEdge> edges;

	float x1,y1,x2,y2;

 	Voronoi::GraphEdge eee;
	while(vd->getNext(eee)) {
		if ( !( eee.x1<minx || eee.y1<miny || eee.x1>maxx || eee.y1>maxy || 
			   eee.x2<minx || eee.y2<miny || eee.x2>maxx || eee.y2>maxy)) 
			edges.push_back(VDEdge(eee.x1,eee.y1,eee.x2,eee.y2));
	}
	eee.next = NULL;
	// ROS_INFO("voronoi edges num = " << edges.size());

	// remove edges behind scan points
	filterVoronoiEdges(edges, mypts);

	// Get the max. radius of free space around each Voronoi node.
	vector<CenterC> candidates = getFreeSpace(edges, mypts);

	/*	
	std::ofstream ofs2("radius.dat",std::ios::app);
	for(int i=0;i<candidates.size();i++) {
		ofs2 << candidates[i].x1 << " " << candidates[i].y1 << " " << candidates[i].r << "\n";
	}
	ofs2.close();
	std::ofstream ofs3("radius-pts.dat");
	for(int i=0;i<pts.size();i++) {
		ofs3 << pts[i].x << " " << pts[i].y << "\n";
	}
	ofs3.close();
	
	*/

	// find largest circle
	int maxr =0;
	for(int i=0;i<candidates.size();i++) {
		if (candidates[i].r > candidates[maxr].r) {
			maxr = i;
		}
	}

	if (candidates.size() > 0) {
		cx = candidates[maxr].x1;
		cy = candidates[maxr].y1;
		radius = sqrt(candidates[maxr].r);
	} else {
		cx = 0;
		cy = 0;
		radius = -1;
	}	

	candidates.clear();
	edges.clear();
	delete vd;
}





} // namespace Laloc
} // namespace lama


