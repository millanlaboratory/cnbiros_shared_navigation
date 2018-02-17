#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_CPP

#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"

namespace cnbiros {
    namespace navigation {


bool ProximitySectorConverter::FromOccupancyGrid(const nav_msgs::OccupancyGrid& msg,
												 ProximitySector& sectors, float threshold) {
	
	grid_map::GridMap			map;
    grid_map::Position			position;
    geometry_msgs::PointStamped map_point;
  
	// Convert Occupancy Grid to GridMap 
	// (for conveniency, to be changed in the future)
    if(grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "costmap", map) == false) {
		ROS_ERROR("Cannot convert occupancy grid to grid map");
		return false;
    }

    // Resetting current sectors
    sectors.Reset();

	// Updating the frame id with the one of the occupancy grid
	sectors.SetFrameId(msg.header.frame_id);

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

		const grid_map::Index index(*iterator);

		// Convert Index to Position
		if(map.getPosition(index, position) == false) {
		    ROS_ERROR("Cannot get position in grid map. Skipping..");
		    continue;
		}

		// Check if the value is greater than threshold
		if(map.at("costmap", index) >= threshold) {

			// Convert into geometry point
		    map_point.header.frame_id = msg.header.frame_id;
		    map_point.point.x = position(0);
		    map_point.point.y = position(1);
		    map_point.point.z = 0.0f;
		 
			// If in the front, update the sectors.
			if(map_point.point.x > 0.0f) {
				sectors.SetByCartesian(map_point.point.x, -map_point.point.y);
			}
		}
    }

	return true;
}

bool ProximitySectorConverter::FromPoint(const geometry_msgs::PointStamped& msg, ProximitySector& sectors) {

	// Reset current sectors
	sectors.Reset();

	// Updating the frame id with the one of the occupancy grid
	sectors.SetFrameId(msg.header.frame_id);
	
	// If in the front, update the sectors.
	if(msg.point.x > 0.0f) {
		sectors.SetByCartesian(msg.point.x, -msg.point.y);
	}

	return true;
}

bool ProximitySectorConverter::FromMessage(const cnbiros_shared_navigation::ProximitySectorMsg& msg,
								  ProximitySector& sectors) {

	// Reset current sectors
	sectors.Reset();

	// Re-Initialize sectors according to the incoming message
	sectors.SetFrameId(msg.header.frame_id);
	sectors.SetResolution(msg.nsectors);
	sectors.SetMinAngle(msg.min_angle);
	sectors.SetMaxAngle(msg.max_angle);

	// Copy the sector values
	sectors.SetValues(msg.values);

	return true;
}

bool ProximitySectorConverter::ToMessage(const ProximitySector& sectors, 
							cnbiros_shared_navigation::ProximitySectorMsg& msg) {

    msg.header.frame_id	= sectors.GetFrameId();
    msg.header.stamp	= ros::Time::now();
    msg.values			= sectors.GetValues();
    msg.min_angle		= sectors.GetMinAngle();
    msg.max_angle		= sectors.GetMaxAngle();
    msg.nsectors		= sectors.GetResolution();
    msg.step			= (msg.max_angle - msg.min_angle)/(float)msg.nsectors;

	return true;
}


	}
}

#endif
