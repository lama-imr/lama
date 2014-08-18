
#include "laloc_utils.h"
#include <math.h>
#include <fstream>
#include <iostream>
#include <cstdlib>

namespace Lama {
namespace Laloc {

using std::cerr;
using std::vector;
using std::ofstream;
using std::string;


/** make list of point from current scan. point far then threshold rt
  * are skipped
 */
vector<SPoint> cutScan(const vector<double> &scan,
		const double maxPhi, const double rt) {

	vector<SPoint> p;

	const int sSize = scan.size();
	
	SPoint tmp;
	for(int i=0;i<sSize;i++) {
		if (scan[i] <= rt) {
			tmp.x = scan[i]*cos(i*maxPhi/sSize);
			tmp.y = scan[i]*sin(i*maxPhi/sSize);
			p.push_back(tmp);
		}
	}
	return p;

}

/** save scan to gnuplot
  */
void saveScan(const char *name, const vector<double> &range, const double maxPhi) {
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


void saveScan(const char *filename, const vector<SPoint> &pts) 
{
	ofstream ofs(filename);

	for(int i=0;i<pts.size();i++)
		ofs << pts[i].x << " " <<pts[i].y<< "\n";
	ofs.close();
}

void setRandom(){

	time_t t;
	
	time(&t);
	srand(t);

}

void getTime(struct rusage *t){
	getrusage(RUSAGE_SELF,t);
}

/**
 * vraci rozdil casu mezi two a one (two-one). vraci v sec
 */
double getTime(struct rusage one, struct rusage two) {

	const unsigned long as = one.ru_utime.tv_sec;
	const unsigned long bs = two.ru_utime.tv_sec;
	const unsigned long aus = one.ru_utime.tv_usec;
	const unsigned long bus = two.ru_utime.tv_usec;

	return (double)((double)bs-(double)as) + 
		(double)((double)bus-(double)aus)/1000000.0;

}

vector<double> toDoubles(const string &s) {

	vector<double> tmp;

	const int n = s.size();
	char tmps[200];
	int i,j;
	i = 0;
	double d;

	while(i<n) {
		while(i<n && s[i] == ' ') i++;
		if (i>=n) return tmp;
		j = i;
		while(j<n && s[j] != ' ') j++;
		
		for(int k=i;k<j;k++)
			tmps[k-i]=s[k];
		tmps[j-i] = '\0';
		sscanf(tmps,"%lf",&d);
		tmp.push_back(d);
		i = j;
	}
	return tmp;
}


void saveDoubles(const char *filename, const std::vector<double> &values) {
	ofstream ofs(filename);

	for(int i=0;i<values.size();i++) {
		ofs << values[i] << "\n";
	}
	ofs.close();
}

} // namespace Laloc
} // namespace Lama

