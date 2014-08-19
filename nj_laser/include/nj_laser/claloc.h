#ifndef _NJ_LASER_CLALOC_H_
#define _NJ_LASER_CLALOC_H_

#include <iostream>
#include <vector>
#include <utility>

#include <nj_laser/cross_detect.h>
/*
 * localization module based on laser detection of a cross
 */

namespace lama {
namespace Laloc {


typedef enum _SimilarityType {
  NCC_FFT = 0,
  POLYGON_BASED_SIMILARITY,
  NCC_RANGE,
  SHAPE_SIM,	// base of Fourier of tangents space of a polygon
  NCC_FFT_FROM_SCAN
} SimilarityType;

class CLaloc {

	public:

    CLaloc();
    ~CLaloc();

    void setDescriptor(const std::vector<double> &scan, const double minPhi, const double maxPhi);



    void crossDetect(const std::vector<double> &scan, const double minPhi, const double maxPhi);

    double getCrossCenterX() const;
    double getCrossCenterY() const;
    double getCrossRadius() const;

    int getNumExits() const;
    double getExitAngle(const int i) const;
    double getExitWidth(const int i) const;
    int getDescriptorFFTSize() const;

    const std::vector<double> & getDescriptor() const;
    const std::vector<double> & getCrossDescriptor() const;

    int getAngleShift(const std::vector<double> &des) const;

    std::string getInitMessage() const;
    std::string getLocalizeMessage(const double xpos = 0, const double ypos = 0) const;
    std::string getFinishMessage() const;

    std::string getLocalizedMessage(const double prob, const double shift) const;

    std::string getLocalizedMessage(const std::vector<double> &vertex) const;

    int getAngleShiftV(const std::vector<double> &vertex) const;
    double getSimilarityV(const std::vector<double> &vertex) const;

    std::string getEdgesMessage() const;

    double getSimilarityNCCFFT_fromScan(const std::vector<double> &desA, const std::vector<double> &desB) const;


    std::vector< std::pair<double,double> > inverseFft(const std::vector<double> &fft) const;
    std::vector<double> inverseFftRange(const std::vector<double> &fft) const;

    void setMaxPhi(const double phi);
    double getMaxPhi(const double phi) const;

    double getBeamAngleSize() const;
    double getBeamOffset() const;
    double getBeamResolution() const;

    void setBeamAngleSize(const double size);
    void setBeamOffset(const double offset);
    void setBeamResolution(const double resolution);

    int getDescriptorSize() const;

  private:

    // lenght of fft of laser signal
    const static int descriptorFFTSize;

    // distance threshold .. longer lines are considered frontiers
    const static double dt;

    // max angle of scan, in radian
    double maxScanPhi;

    // fft descriptor of a place
    std::vector<double> actDescriptor;

    // [cx,cy,cr, angle1, angle2 ... ]
    // descriptor of a cross
    // if no cros .. cx=cy=cr=0
    std::vector<double> actCrossdescriptor;
    std::vector<SFrontier> actFrontiers;

    SimilarityType similarityType;

    // size of laser range finder beam; in radian
    double beamAngleSize; 

    // offset of first ray in laser beam, eg. -40*M_PI/180.0 for hockuoy, also
    // angle of first ray; in radian
    double beamOffset;

    // resolution of laser range finder in degrees, eg. 0.25 for sick
    double beamResolution;

    std::vector<double> getDataFromVertex(const std::vector<double> &vertex) const;
};

} // namespace Laloc
} // namespace lama

#endif // _NJ_LASER_CLALOC_H_

