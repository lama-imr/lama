/*
 * Obstacle avoidance with LaserScan.
 *
 * Drive the robot while avoiding obstacles:
 * - onTraverse and onContinue: go more or less forward depending
 *     on obstacle position. The action never stops by itself.
 *
 * Interaction with the map (created by this jockey):
 * - none.
 *
 * Interaction with the map (created by other jockeys):
 * - none.
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs/LaserScan, "~base_scan", laser-scan at the front of the
 *   robot.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 * - geometry_msgs/Twist, "~cmd_vel", set velocity.
 *
 * Services used (other than map-related):
 * - none.
 *
 * Parameters:
 * - ~robot_width, Float, NO_DEFAULT, robot width.
 * - ~min_distance, Float, 2 * robot_width, if an obstacle is closer than this,
 *     turn and don't go forward (m/s).
 * - ~long_distance, Float, 5 * robot_width, if no obstacle within this
 *     distance, go straight (m/s).
 * - ~turnrate_collide, Float, 0.4, turn rate when obstacle closer than
 *     min_distance_ (rad/s).
 * - ~max_vel, Float, 1.0, linear velocity without obstacle (m/s).
 * - ~vel_close_obstacle, Float, 0.5, linear velocity if obstacle between
 *     min_distance and long_distance (m/s).
 * - ~turnrate_factor, Float, 0.9, if obstacle closer than long_distance
 *     turnrate = -turnrate_factor_ * mean(lateral_position_of_obstacle)
 *     (rad.m^-1.s^-1).
 */

#ifndef NJ_OA_LASER_JOCKEY_H
#define NJ_OA_LASER_JOCKEY_H

#include <cmath>

#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <lama_jockeys/navigating_jockey.h>

namespace lama {
namespace nj_oa_laser {

class Jockey : public NavigatingJockey
{
  public :

    Jockey(const std::string& name, const double robot_width);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    double getRobotWidth() const {return robot_width_;}
    void setRobotWidth(const double value) {robot_width_ = value;}

    double getMinDistance() const {return min_distance_;}
    void setMinDistance(const double value) {min_distance_ = value;}

    double getLongDistance() const {return long_distance_;}
    void setLongDistance(const double value) {long_distance_ = value;}

  protected:

    void manageTwist(const sensor_msgs::LaserScan& scan);

    // Subscribers and publishers.
    ros::Publisher pub_twist_;

  private :

    void handleLaser(const sensor_msgs::LaserScanConstPtr& msg);

    // Parameters shown outside.
    double robot_width_;  //!> (m), robot width.
    double min_distance_;  //!> (m), if an obstacle is closer than this, turn and don't go forward.
    double long_distance_;  //!> (m), if no obstacle within this distance, go straight.
    double turnrate_collide_;  //!> (rad/s), turn rate when obstacle closer than min_distance_.
    double max_vel_;  //!> (m/s), linear velocity without obstacle.
    double vel_close_obstacle_;  //!> (m/s), linear velocity if obstacle between min_distance_ and long_distance_.
    double turnrate_factor_;  //!> (rad.m^-1.s^-1, > 0), if obstacle closer than long_distance_,
                              //!> turnrate = -turnrate_factor_ * mean(lateral_position_of_obstacle).
};

} // namespace nj_oa_laser
} // namespace lama

#endif // NJ_OA_LASER_JOCKEY_H
