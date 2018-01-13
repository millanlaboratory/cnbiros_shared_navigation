#ifndef CNBIROS_NAVIGATION_COSTMAP_TO_POLARGRID_HPP
#define CNBIROS_NAVIGATION_COSTMAP_TO_POLARGRID_HPP

#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "cnbiros_wheelchair_navigation/PolarGrid.h"
#include "cnbiros_wheelchair_navigation/SectorGrid.h"
#include <costmap_2d/cost_values.h>

namespace cnbiros {
    namespace navigation {

class CostMapToPolarGrid {

    public:
	CostMapToPolarGrid(void);
	virtual ~CostMapToPolarGrid(void);
	
    protected:
	virtual void callback(const nav_msgs::OccupancyGrid& data_in);
	void init_sectors(unsigned int nsectors, float min_angle, float max_angle);
	void fill_sectors(float angle, float radius);
	void reset_sectors(void);

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


	unsigned int nsectors_;
	float sector_min_angle_;
	float sector_max_angle_;
	float sector_step_;
	std::vector<float> sectors_;


};



    }
}

#endif
