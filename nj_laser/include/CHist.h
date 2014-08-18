
#ifndef CHIST_H__
#define CHIST_H__

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>

namespace Lama {
namespace Laloc {

class CHist {

	public:
		CHist(const std::vector<double> &range):
   			range(range),values(range.size(),0)
		{
		}

		CHist(const double from, const double to, const double step) {
			const int s = (int)lround((to - from) / step);
			for(int i=0;i<s;i++) {
				range.push_back(from+i*step);
				values.push_back(0);
			}
			
		}

		~CHist() {
			range.clear();
			values.clear();
		}

		CHist(const CHist &h) :
			range(h.range),values(h.values)
		{
		}


		void add(const double value) {
			for(uint i = 0; i < range.size() - 1; i++)
			{
				if (range[i] <= value && value < range[i + 1])
				{
					values[i]++;
					return;
				}
			}
			values[range.size() - 1]++;
		}

		void add(const std::vector<double> &values){
			for(uint i=0;i<values.size();i++)
				add(values[i]);
		}

		std::vector<int> getHist() const {
			return values;
		}

		std::vector<double> getRange() const {
			return range;
		}

		void save(const char *filename) const {
			std::ofstream ofs(filename);
			uint i=0;
			ofs << "#histogram <histValue> <binFrom> <binTo>\n";
			for(i=0;i<values.size()-1;i++)
				ofs << values[i] << " " << range[i]<<" "<<range[i+1]<<"\n";
			ofs << values[values.size()-1] << " " << range[range.size()-1] <<"\n";

		}

		/* Return the bin lower bound at index i
		 */
		double getRangeLo(const int i) const {
			if (i>=0 && i<range.size())
				return range[i];
			return 0;
		}

		/* Return the bin upper bound at index i
		 */
		double getRangeHi(const int i) const {
			if (i>=0 && i<range.size()-1)
				return range[i+1];
			return 0;
		}

		/* Return the index of the bin containing the most values
		 */
		int getMaxBin() const {
			int m = 0;
			for(int i=0;i<values.size();i++)
				if (values[i] > values[m])
					m = i;
			return m;
		}


		friend std::ostream &operator<<(std::ostream &os, const CHist &h);



	private:
		std::vector<double> range;
		std::vector<int> values;

};

		
std::ostream &operator<<(std::ostream &os, const CHist &h) {
	os << "CHist("<<h.values.size()<<":";
	for(int i=0;i<h.values.size()-1;i++)
		os << "["<<h.range[i]<<","<<h.range[i+1]<<"]="<<h.values[i]<<"\n";

	os << h.range[h.range.size()-1]<<", ]="<<h.values[h.values.size()-1]<<"\n";
	return os;
}


} // namespace Laloc
} // namespace Lama

#endif

