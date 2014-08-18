
#include <utility>
#include <vector>
#include <math.h>
#include <list>
#include <algorithm>
#include <iostream>
#include <fstream>



#include "laloc_utils.h"
#include "shapeSim.h"

namespace Lama {
namespace Laloc {

using std::pair;
using std::vector;
using std::list;
using std::make_pair;
using std::cerr;
using std::ofstream;

typedef SPoint Point;



vector<Point> rangeToPoint( const vector<double> &range, const double maxPhi) 
{
	if (range.size() == 0)
		return vector<Point>();

	vector<Point> result;
	result.reserve(range.size());

	double a;
	for(int i=0;i<range.size();i++) {
		a = i*maxPhi/range.size();
		result.push_back(Point(range[i]*cos(a),range[i]*sin(a)));
	}
	return result;
}

struct S {
		int idx;
		double r;
		S(const int i, const double rel):idx(i),r(rel) {}
};

template<typename T>
class myLess {
	public:
	bool operator()(const T &a, const T &b) {
		return a.r < b.r;
	}
};

inline double getRelevance(const Point &pi, const Point &pj, const Point &pk) {
	double rel, dx,dy;
	dx = pi.x - pj.x;
	dy = pi.y - pj.y;
	rel = sqrt(dx*dx + dy*dy);
	dx = pj.x - pk.x;
	dy = pj.y - pk.y;
	rel += sqrt(dx*dx + dy*dy);
	dx = pi.x - pk.x;
	dy = pi.y - pk.y;
	rel -= sqrt(dx*dx+ dy*dy);
	return fabs(rel);
}

double getRelevance(const list<S> &s, list<S>::const_iterator i, const vector<Point> &pts) {
	list<S>::const_iterator pred = i;
	list<S>::const_iterator succ = i;
	pred--;
	succ++;

	double rel = -1;
	if (i != s.begin() && succ != s.end()) {
		rel = getRelevance(pts[pred->idx],pts[i->idx],pts[succ->idx]);
	} 
	return rel;
}

vector<Point> filterRelevance(const vector<Point> &pts, const double maxR) {
	
	if (pts.size() < 3)
		return pts;

	list<S> r;
	double dx,dy,rel;
	double maxRel = -1;
	for(int j=0;j<pts.size();j++) {
		if (j > 0 && j < pts.size()-1) {
			rel = getRelevance(pts[j-1], pts[j], pts[j+1]);
		} else 
			rel = -1;
		r.push_back(S(j,rel));
		if (rel > maxRel || maxRel == -1)
			maxRel = rel;
	}
	r.front().r = 10*maxRel;
	r.back().r = 10*maxRel;

//	std::ofstream ofs("rel.his");
//	for(list<S>::const_iterator i = r.begin(); i != r.end(); i++)
//		ofs << i->idx << " " << i->r << "\n";
//	ofs.close();

	double minR,rr;
	do {
		list<S>::iterator me = std::min_element(r.begin(),r.end(),myLess<S>());
		if (me == r.end()) {
			break;
		}
		minR = me->r;
		if (me != r.begin() && me != r.end()) {
			list<S>::iterator i = me;
			i--;
			list<S>::iterator j = me;
			j++;
			r.erase(me);
			if (i != r.begin() && j != r.end()) {
				if (i != r.begin() && i != r.end()) {
					rr = getRelevance(r,i,pts);
					if (rr >= 0)
						i->r = rr;
				}
				if (j != r.begin() && j != r.end()) {
					rr = getRelevance(r,j,pts);
					if (rr >= 0)
						j->r = rr;
				}
			} else {
				break;
			}
		}
		if (r.size() <= 3)
			break;
	} while (minR < maxR);
	
	vector<Point> res;
	res.reserve(r.size());
	for(list<S>::const_iterator i = r.begin(); i != r.end(); i++)
		res.push_back(pts[i->idx]);

	return res;
}



double getAngleRad(const Point &p1, const Point &p2, const Point &p3) {

	const double a = sqrt( 
			(p1.x-p2.x)*(p1.x-p2.x) +
		(p1.y-p2.y)*(p1.y-p2.y));
	
	const double b = sqrt(
			( p2.x - p3.x)*( p2.x - p3.x ) +
		( p2.y - p3.y )*( p2.y - p3.y )
		);
			
	const double c = sqrt(
			( p1.x - p3.x)*( p1.x - p3.x ) +
		( p1.y - p3.y )*( p1.y - p3.y )
		);


	if (a == 0 || b == 0) {
		return 0;
	}	

	double aco =  acos((a*a+b*b-c*c)/(2*a*b));

	if (isnan(aco)) {
		const double bn = p3.x - p1.x;
		const double an = p1.y  - p3.y;
		const double cn = -an* p1.x - bn*p1.y;
		const double dist = fabs(an*p2.x+bn*p2.y+cn)/sqrt(an*an + bn*bn);
		if (dist < 0.05) {
			aco = 0;
		} else {
//			std::cerr << "Point near collinera: d="<<dist<<"\n";
			exit(0);
		}
	}
	return aco;
}

inline double distance(const Point &p1, const Point &p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return sqrt(dx*dx + dy*dy);
}

//return true if p3 is left from line (p1,p2)
inline bool isLeft(const Point &p1, const Point &p2, const Point &p3) {
	return ( (p2.x - p1.x)*(p3.y-p1.y) - (p3.x-p1.x)*(p2.y-p1.y) ) > 0;
}

double turningAngle(const Point &p1, const Point &p2, const Point &p3) {
	
	double a = getAngleRad(p1,p2,p3);
	if (!isLeft(p1,p2,p3))
		a = -a;

	return a;
}

vector< pair<double,double> > toTangetSpace(const vector<Point> &pts) {

	vector< pair<double, double> > res;

	if (pts.size() < 3)
		return res;

	int i,k;
	double a,d;
	for(int j=1;j<pts.size()-1;j++) {
		i = j-1;
		k = j+1;
		a = turningAngle(pts[i],pts[j],pts[k]);
		d = distance(pts[i],pts[j]);

		res.push_back(make_pair(d,a));
	}
		
	return res;

}

template<typename Iter>
void normalize(Iter begin, Iter end) {
	double d = 0;
	
	for(Iter i = begin; i != end; ++i) {
		d+=i->first;
	}

	if (d != 0) {
		for(Iter i = begin; i != end; ++i) {
			i->first/=d;
		}
	}
}

vector<double> sampling(const vector< pair<double,double> > &ts, const int samples) {
	vector<double> res;

	if (samples <= 0)
		return res;

	res.reserve(samples);


	int i = 0;
	double px = 0;
	const double dx = 1.0 / (double)samples;	
	double ac = 0;
	double x = 0;
	while(x < 1) {
		res.push_back(ac);
		x += dx;
		if (x - px > ts[i].first) {
			while( x - px > ts[i].first) {
				px += ts[i].first;
				ac += ts[i].second;
				i++;
			}
		}
		if ( i >= ts.size()) {
//			cerr << "*";
			break;
		}
	}
	return res;
}

template<typename Iter>
void scale(Iter begin, Iter end, const double s) {

	for(Iter i = begin; i != end; ++i) {
		*i = Point(i->x*s,i->y*s);
	}
}




} // namespace Laloc
} // namespace Lama

