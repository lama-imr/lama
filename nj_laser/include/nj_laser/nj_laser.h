/* Memory-less navigating jockey based on LaserScan
 */

#ifndef _NJ_LASER_H_
#define _NJ_LASER_H_

#include <sensor_msgs/LaserScan.h>

#include <lama_interfaces/navigating_jockey.h>

#include <nj_laser/claloc.h>
#include <nj_laser/visualization.h>

class NJLaser : public lama::interfaces::NavigatingJockey
{
  public:

    NJLaser(std::string name);

    virtual void onTraverse();
    virtual void onStop();
	virtual void onInterrupt();
	virtual void onContinue();

  private:

	void handleLaser(const sensor_msgs::LaserScan msg);
	
	ros::Subscriber laserHandler_;
	ros::Publisher crossing_marker_;
	ros::Publisher exits_marker_;
	lama::Laloc::CLaloc cl_;
};

#endif // _NJ_LASER_H_

