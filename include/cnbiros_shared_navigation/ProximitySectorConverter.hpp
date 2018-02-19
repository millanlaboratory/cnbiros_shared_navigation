#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>

// Package include
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"

namespace cnbiros {
    namespace navigation {

/*!
 * This class provides some conversions from/to ProximitySector object.
 * The methods are static and can be called without instanciation.
 */
class ProximitySectorConverter {

	public:

		/*!
		 * Converts a ProximitySector message in a ProximitySector object.
		 * @param[in] msg The ProximitySector message
		 * @param[out] sectors The ProximitySector object
		 * @return True if succesful, false otherwise
		 */
		static bool FromMessage(const cnbiros_shared_navigation::ProximitySectorMsg& msg, ProximitySector& sectors);

		/*!
		 * Converts a OccupancyGrid message in a ProximitySector object. A
		 * threshold on the occupancy grid values can be provided.
		 * @param[in] msg The OccupancyGrid message
		 * @param[out] sectors The ProximitySector object
		 * @param[in] threshold Value for thresholding the occupancy grid before
		 * the conversion (default: 100.0f).
		 * @return True if succesful, false otherwise.
		 */
		static bool FromOccupancyGrid(const nav_msgs::OccupancyGrid& msg, ProximitySector& sectors, 
		 					   float threshold=100.0f);

		/*!
		 * Converts a PointStamped message in a ProximitySector object.
		 * @param[in] msg The PointStamped message
		 * @param[out] sectors The ProximitySector object
		 * @return True if succesful, false otherwise.
		 */
		static bool FromPoint(const geometry_msgs::PointStamped& msg, ProximitySector& sectors);

		/*!
		 * Converts a ProximitySector object in a ProximitySector message.
		 * @param[in] sectors The ProximitySector object
		 * @param[out] msg The ProximitySector message
		 * @return True if succesful, false otherwise.
		 */
		static bool ToMessage(const ProximitySector& sectors, cnbiros_shared_navigation::ProximitySectorMsg& msg);
};


	}
}
#endif
