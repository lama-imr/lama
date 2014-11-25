/*
 *
 *  Created on: May 30, 2013
 *      Author: Vlaimir Petrik vladko.petrik@gmail.com
 */

#ifndef FEATURENAV_BASE_SEGMENT_LEARNER_H
#define FEATURENAV_BASE_SEGMENT_LEARNER_H

#include <featurenav_base/Landmark.h>
#include <featurenav_base/segment_processor_base.h>

class SegmentLearner : public SegmentProcessorBase
{
  public:

    SegmentLearner();
    virtual ~SegmentLearner();

    /** \brief Start segment learning process.
     *  \param orientation initial orientation of the robot
     */
    void start_learning(double orientation);

    /** \brief Stop segment learning process.
     *  \details The segment will be saved to the file specified by id.
     *  \param d Distance from the segment start in meters.
     *  \return true if save was successful
     */
    bool stop_learning(double d);

    /** \brief Process image.
     *  \param image Image obtained by the camera.
     *  \param d Distance from the segment start in meters.
     *  \return Number of tracked landmarks.
     */
    int process_image(const cv::Mat& image, double d);

public:

    /** Minimum tracking distance to learn the landmark in meters. Default is 0.02.*/
    double min_landmark_position_dist;
    std::string id; //Path to the segment.

private:
    std::vector<Landmark> landmarks_; //Tracked landmarks
};

#endif // FEATURENAV_BASE_SEGMENT_LEARNER_H

