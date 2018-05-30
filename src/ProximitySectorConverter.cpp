#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTORCONVERTER_CPP

#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"

namespace cnbiros {
    namespace navigation {


bool ProximitySectorConverter::FromOccupancyGrid(const nav_msgs::OccupancyGrid& msg,
												 tf::TransformListener* listener,
												 ProximitySector& sectors, 
												 float threshold) {
	grid_map::GridMap			map;
    grid_map::Position			position;
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped sector_point;
  
	// Convert Occupancy Grid to GridMap 
	// (for conveniency, to be changed in the future)
    if(grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "costmap", map) == false) {
		ROS_ERROR("Cannot convert occupancy grid to grid map");
		return false;
    }

    // Resetting current sectors
    sectors.Reset();

	// Updating the frame id with the one of the occupancy grid

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

			// Try to transform from occupancy grid frame to sector frame
			try {
				listener->waitForTransform(sectors.GetFrameId(), map_point.header.frame_id, 
											ros::Time(0), ros::Duration(10.0) );
				listener->transformPoint(sectors.GetFrameId(), map_point, sector_point);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s", ex.what());
				continue;
			}

			// If in the front, update the sectors.
			if(sector_point.point.x > 0.0f) {
				sectors.SetByCartesian(sector_point.point.x, -sector_point.point.y);
			}
		}
    }

	return true;
}

bool ProximitySectorConverter::FromPoint(const geometry_msgs::PointStamped& msg, 
										 tf::TransformListener* listener, 
										 ProximitySector& sectors) {

	geometry_msgs::PointStamped  sector_point;

	// Reset current sectors
	sectors.Reset();

	// Updating the frame id with the one of the occupancy grid

	// Try to transform from point frame to sector frame
	try {
		listener->waitForTransform(sectors.GetFrameId(), msg.header.frame_id, 
									ros::Time(0), ros::Duration(10.0) );
		listener->transformPoint(sectors.GetFrameId(), msg, sector_point);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		return false;
	}

	// If in the front, update the sectors.
	if(sector_point.point.x > 0.0f) {
		sectors.SetByCartesian(sector_point.point.x, -sector_point.point.y);
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


bool ProximitySectorConverter::FromLaserScan(const sensor_msgs::LaserScan& msg, 
											tf::TransformListener* listener,
											ProximitySector& sectors) {

    geometry_msgs::PointStamped scan_point;
    geometry_msgs::PointStamped sector_point;
	
	float scan_angle;
	float scan_inc;

	sectors.Reset();
	
	scan_angle = msg.angle_min;
	scan_inc   = msg.angle_increment;
	for(auto it=msg.ranges.begin(); it!=msg.ranges.end(); ++it) {
		printf("cangle = %f\n", scan_angle);
		scan_point.header.frame_id = msg.header.frame_id;
		scan_point.point.x = (*it)*std::sin(scan_angle);
		scan_point.point.y = (*it)*std::cos(scan_angle);
		scan_point.point.z = 0.0f;
		scan_angle += scan_inc;

		if(std::isinf( (*it) ) == true)
			continue;

		try {
			listener->waitForTransform(sectors.GetFrameId(), scan_point.header.frame_id,
									   ros::Time(0), ros::Duration(10.0) );
			listener->transformPoint(sectors.GetFrameId(), scan_point, sector_point);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
			continue;
		}

		try {
			sectors.SetByCartesian(sector_point.point.x, -sector_point.point.y);	
		} catch (std::out_of_range& ex) {
			printf("%s\n", ex.what());
			printf("Sector point: (%f, %f)\n", sector_point.point.x, -sector_point.point.y);
			printf("Radius: %f\n", (*it));
		}

	}

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
    msg.step			= sectors.GetStep();

	return true;
}


	}
}

#endif
