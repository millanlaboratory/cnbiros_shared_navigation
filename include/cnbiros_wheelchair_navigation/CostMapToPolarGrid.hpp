#ifndef CNBIROS_NAVIGATION_COSTMAP_TO_POLARGRID_HPP
#define CNBIROS_NAVIGATION_COSTMAP_TO_POLARGRID_HPP

#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "cnbiros_wheelchair_navigation/PolarGrid.h"

namespace cnbiros {
    namespace navigation {

class CostMapToPolarGrid {

    public:
	CostMapToPolarGrid(void);
	virtual ~CostMapToPolarGrid(void);
	
    protected:
	virtual void callback(const nav_msgs::OccupancyGrid& data_in);


    protected:
	ros::NodeHandle nh_;
  	ros::NodeHandle private_nh_;
	tf::TransformListener listener;
	
	ros::Subscriber	    sub_;
	ros::Publisher	    pub_;
	std::string stopic_;
	std::string ptopic_;

	float obstacle_marker_;
	std::string polar_frame_id_;


};



    }
}

#endif
