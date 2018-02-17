#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_HPP

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf/transform_listener.h>

// Package include
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"

namespace cnbiros {
    namespace navigation {

class ProximitySectorConverter {

	public:
		static bool FromOccupancyGrid(const nav_msgs::OccupancyGrid& msg, ProximitySector& sectors, 
		 					   float threshold=100.0f);
		static bool FromPoint(const geometry_msgs::PointStamped& msg, ProximitySector& sectors);
		static bool FromMessage(const cnbiros_shared_navigation::ProximitySectorMsg& msg, ProximitySector& sectors);

		static bool ToMessage(ProximitySector& sectors, cnbiros_shared_navigation::ProximitySectorMsg& msg);
};


	}
}
#endif
