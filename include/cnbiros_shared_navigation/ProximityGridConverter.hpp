#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDCONVERTER_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDCONVERTER_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>

// Package include
#include "cnbiros_shared_navigation/ProximityGrid.hpp"
#include "cnbiros_shared_navigation/ProximityGridMsg.h"

namespace cnbiros {
    namespace navigation {

class ProximityGridConverter {

	public:

		static bool FromMessage(const cnbiros_shared_navigation::ProximityGridMsg& msg, ProximityGrid& grid);

		static bool FromPoint(const geometry_msgs::PointStamped& msg, ProximityGrid& grid,
							  tf::TransformListener* listener);

		static bool FromLaserScan(const sensor_msgs::LaserScan& msg, ProximityGrid& grid,
								  tf::TransformListener* listener);
								
		static bool ToMessage(const ProximityGrid& grid, cnbiros_shared_navigation::ProximityGridMsg& msg);

		static bool ToLaserScan(const ProximityGrid& grid, sensor_msgs::LaserScan& scan);
};


	}
}
#endif
