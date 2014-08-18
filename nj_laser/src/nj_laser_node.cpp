/**
  * Large Map 
  * Laser based memoryless (reactive) navigating  jockey
  *
  */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <lama_interfaces/lama_navigating_jockey.h>
#include <lama_interfaces/lmi_laser_descriptor_set.h>
#include <lama_interfaces/lmi_laser_descriptor_get.h>

#include <nj_laser/claloc.h>

namespace li = lama_interfaces;
ros::Publisher marker;
ros::Publisher crossingMarker;
//ros::Publisher draw;
//ros::Publisher drawRef;
sensor_msgs::LaserScan first;
sensor_msgs::LaserScan last;
std::vector<sensor_msgs::LaserScan> descriptor;

//bool assigned;
//bool learning = false;
bool navigating = false;
bool interrupted = false;
//int angleHistBin=100;
//float max_error = 0.2;
//float max_distance = 0.3;
//float min_distance = 0.05;
//int navigation_index = 0;
Lama::Laloc::CLaloc cl;


void handleLaser(sensor_msgs::LaserScan msg) {
   ROS_INFO("LEARNING laser arrived %lu", msg.ranges.size());
   std::vector<double> ranges;
   for (uint i =0; i < msg.ranges.size(); i++){
      ranges.push_back(msg.ranges[i]);
   }

   cl.crossDetect(ranges, msg.angle_min, msg.angle_max);
   std::vector<double> desc = cl.getCrossDescriptor();
   ROS_INFO("CROSS x: %.3f, y: %.3f, r: %.3f, nroads: %lu", desc[0], desc[1], desc[2], desc.size() - 3);

   //visualization_msgs::MarkerArray ma;
   // Visualization: a sphere at detected crossing center
   visualization_msgs::Marker m;
   m.header.frame_id = "/base_laser_link";
   m.ns = "crossing_center";
   m.type = visualization_msgs::Marker::SPHERE;
   m.pose.position.y = cl.getCrossCenterX(); //!! x <--> y
   m.pose.position.x = cl.getCrossCenterY(); //!! x <--> y
   m.pose.position.z = 0;
   m.pose.orientation.w = 1.0;
   m.scale.x = cl.getCrossRadius();
   m.scale.y = cl.getCrossRadius();
   m.scale.z = 1;
   m.color.r = 1.0;
   m.color.g = 1.0;
   m.color.a = 0.5;
   marker.publish(m);
 
   // Visualization: a line at each detected road
   visualization_msgs::Marker ml;
   ml.header.frame_id = "/base_laser_link";
   ml.ns = "road_direction";
   ml.type = visualization_msgs::Marker::LINE_LIST;
   ml.pose.orientation.w = 1.0;
   ml.scale.x = 0.1;
   ml.color.r = 0.0;
   ml.color.g = 0.0;
   ml.color.b = 1.0;
   ml.color.a = 0.5;
   for(uint i = 3; i < desc.size(); ++i)
   {
           geometry_msgs::Point p;
           ml.points.push_back(p);
           p.x = cl.getCrossRadius() * cos(desc[i]);
           p.y = cl.getCrossRadius() * sin(desc[i]);
	   ROS_INFO("ROAD %f", desc[i]);
           ml.points.push_back(p);
   }
   crossingMarker.publish(ml);

   if (navigating && !interrupted) {

   }
}

bool navigationCallback(li::lama_navigating_jockeyRequest& request, li::lama_navigating_jockeyResponse& response){
   li::lmi_laser_descriptor_get ldg;
   switch (request.request_action.action)  {
      case 0: 	// stop learn
	 ROS_INFO("STOP");
	 navigating = false;
	 interrupted = false;
	 response.actual_state.state = 0;
	 break;
      case 1:  	// start learn
	 if (!interrupted) {
	    ROS_INFO("navigation START");
	    descriptor.clear();
	    ldg.request.id = request.descriptor;
	    ros::service::call("lmi_laser_descriptor_getter",ldg);
	    descriptor = ldg.response.descriptor;
	    //assigned = false;
	    navigating = true;
	    response.actual_state.state = 1;
	 } else {
	    response.actual_state.state = 2;
	 }

	 break;
   }
   return true;
}


int main(int argc, char **argv) {

   ros::init(argc, argv, "laser_jockey");
   //assigned = false;
   ros::NodeHandle n;
   ros::Subscriber laserHandler = n.subscribe<sensor_msgs::LaserScan> ("base_scan", 50, handleLaser);
   marker = n.advertise<visualization_msgs::Marker> ("cross_marker",50, true);
   crossingMarker = n.advertise<visualization_msgs::Marker> ("cross_line_marker", 50, true);
   ros::ServiceServer navigatingJockey = n.advertiseService ( "navigating_jockey", navigationCallback);

   //   pub = n.advertise<geometry_msgs::Twist> ("nav_cmd",1,false);
   //  draw = n.advertise<geometry_msgs::PolygonStamped> ("draw",1,false);
   // drawRef = n.advertise<geometry_msgs::PolygonStamped> ("draw_reference",1,false);
   ROS_INFO("started");
   ros::spin();

}

#undef intro
