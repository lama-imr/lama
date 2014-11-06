#include <nj_oa_laser/jockey.h>

namespace lama {
namespace nj_oa_laser {

Jockey::Jockey(const std::string& name, const double robot_width) :
  NavigatingJockey(name),
  robot_width_(robot_width),
  min_distance_(2 * robot_width),
  long_distance_(5 * robot_width),
  turnrate_collide_(0.4),
  max_vel_(1.0),
  vel_close_obstacle_(0.5),
  turnrate_factor_(0.9)
{
  private_nh_.getParam("robot_width", robot_width_);
  private_nh_.getParam("min_distance", min_distance_);
  private_nh_.getParam("long_distance", long_distance_);
  private_nh_.getParam("turnrate_collide", turnrate_collide_);
  private_nh_.getParam("max_vel", max_vel_);
  private_nh_.getParam("vel_close_obstacle", vel_close_obstacle_);
  private_nh_.getParam("turnrate_factor", turnrate_factor_);
}

void Jockey::onTraverse()
{
  ROS_DEBUG("%s: Received action TRAVERSE or CONTINUE", ros::this_node::getName().c_str());

  ros::Subscriber laser_handler = private_nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 1, &Jockey::handleLaser, this);
  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ros::Rate r(100);
  while (true)
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    ros::spinOnce();
    r.sleep();
  }

  pub_twist_.shutdown();
}

void Jockey::onStop()
{
  ROS_DEBUG("%s: Received action STOP or INTERRUPT", ros::this_node::getName().c_str());
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onInterrupt()
{
  ROS_DEBUG("%s: Received action INTERRUPT", ros::this_node::getName().c_str());
  onStop();
}

void Jockey::onContinue()
{
  ROS_DEBUG("%s: Received action CONTINUE", ros::this_node::getName().c_str());
  onTraverse();
}

void Jockey::handleLaser(const sensor_msgs::LaserScanConstPtr& msg)
{
  manageTwist(*msg);
}

/* Compute the Twist message and publish it
*/
void Jockey::manageTwist(const sensor_msgs::LaserScan& scan)
{
   bool collide = false;
   bool go_straight = true;
   double sum_y = 0;
   unsigned int count_y = 0;
   double sum_y_colliding = 0;
   const double half_width = robot_width_ / 2;
   const double long_wdith = 1.5 * half_width;

   double x;
   double y;
   for (unsigned int i = 0; i < scan.ranges.size() / 2; ++i)
   {
     const double angle = angles::normalize_angle(scan.angle_min + i * scan.angle_increment);
     if ((angle < -M_PI_2) || (angle > M_PI_2))
     {
       // Do no consider a beam directed backward.
       continue;
     }

     x = scan.ranges[i] * std::cos(angle);
     y = scan.ranges[i] * std::sin(angle);

     if ((x < min_distance_)  && (-half_width < y) && (y < half_width))
     {
       collide = true;
       sum_y_colliding += x;   
     } 

     if ((x < long_distance_)  && (-long_wdith < y) && (y < long_wdith))
     {
       go_straight = false;
     }
     sum_y += y;
     count_y++;  
   }

   double speed;
   double turnrate;
   if (collide)
   { 
     speed = 0;
     if (sum_y_colliding < 0)
     { 
       turnrate = -turnrate_collide_;
     }
     else
     {
       turnrate = turnrate_collide_;
     }      
   }
   else if (go_straight)
   {
     speed = max_vel_;
     turnrate = 0;
   }
   else
   {
     speed = vel_close_obstacle_;
     turnrate = -turnrate_factor_ * sum_y / ((double) count_y);
   }

   geometry_msgs::Twist twist;
   twist.linear.x = speed;
   twist.angular.z = turnrate;
   pub_twist_.publish(twist);
}

} // namespace nj_oa_laser
} // namespace lama
