#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDCONVERTER_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDCONVERTER_CPP

#include "cnbiros_shared_navigation/ProximityGridConverter.hpp"

namespace cnbiros {
    namespace navigation {

bool ProximityGridConverter::FromMessage(const cnbiros_shared_navigation::ProximityGridMsg& msg,
								  ProximityGrid& grid) {

	// Reset current sectors
	grid.Reset();

	// Re-Initialize sectors according to the incoming message
	grid.SetAngleLimits(msg.angle_min, msg.angle_max);
	grid.SetAngleIncrement(msg.angle_increment);
	grid.SetRangeLimits(msg.range_min, msg.range_max);
	grid.SetFrame(msg.header.frame_id);

	// Copy the sector values
	grid.SetGrid(msg.ranges);

	return true;
}


bool ProximityGridConverter::FromPoint(const geometry_msgs::PointStamped& msg, 
									   ProximityGrid& grid,
									   tf::TransformListener* listener) {

	geometry_msgs::PointStamped  point_grid;
	std::string frame_id_grid;
	std::string frame_id_point;
	
	// Reset current sectors
	grid.Reset();

	// Get grid frame ids
	frame_id_grid  = grid.GetFrame();
	frame_id_point = msg.header.frame_id;

	// Try to transform from incoming point msg to a point in grid frame
	try {
		listener->waitForTransform(frame_id_grid, frame_id_point, 
									ros::Time(0), ros::Duration(3.0) );
		listener->transformPoint(frame_id_grid, msg, point_grid);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		return false;
	}

	grid.SetSectorByCartesian(point_grid.point.y, point_grid.point.x);

	return true;
}



bool ProximityGridConverter::FromLaserScan(const sensor_msgs::LaserScan& msg, 
											ProximityGrid& grid, 
											tf::TransformListener* listener) {

    geometry_msgs::PointStamped point_scan;
    geometry_msgs::PointStamped point_grid;
	
	float angle;
	std::string frame_id_grid;
	std::string frame_id_scan;

	// Reset the grid
	grid.Reset();

	// Get grid frame ids
	frame_id_grid = grid.GetFrame();
	frame_id_scan = msg.header.frame_id;

	// Fill the point_scan object with the msg header
	point_scan.header = msg.header;

	// Initialize current angle with the minimum angle of the scan message
	angle = msg.angle_min;
	
	// Iterate for each sector of the scan msg
	for(auto it=msg.ranges.begin(); it!=msg.ranges.end(); ++it) {

		point_scan.point.x = (*it)*std::sin(angle+M_PI/2.0f); 
		point_scan.point.y = (*it)*std::cos(angle+M_PI/2.0f);
		point_scan.point.z = 0.0f;

		// Update angle for the next iteration
		angle += msg.angle_increment;

		// Check if the current value is inf, nan
		if(std::isinf( (*it) ) || std::isnan( (*it) ) )
			continue;

		// Check if the current value is outside range_min range_max
		if( (*it) < msg.angle_min || (*it) > msg.angle_max ) 
			continue;
		
		// Try to transform to the grid frame
		try {
			listener->waitForTransform(frame_id_grid, frame_id_scan,
									   ros::Time(0), ros::Duration(3.0) );
			listener->transformPoint(frame_id_grid, point_scan, point_grid);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
			continue;
		}

		grid.SetSectorByCartesian(point_grid.point.x, point_grid.point.y);	
	}

	return true;
}

bool ProximityGridConverter::ToMessage(const ProximityGrid& grid, 
							cnbiros_shared_navigation::ProximityGridMsg& msg) {

    msg.header.frame_id	= grid.GetFrame();
    msg.header.stamp	= ros::Time::now();
    msg.ranges			= grid.GetGrid();
    msg.angle_min		= grid.GetAngleMin();
    msg.angle_max		= grid.GetAngleMax();
    msg.angle_increment	= grid.GetAngleIncrement();
	msg.range_min		= grid.GetRangeMin();
	msg.range_max		= grid.GetRangeMax();

	return true;
}

bool ProximityGridConverter::ToLaserScan(const ProximityGrid& grid, 
										 sensor_msgs::LaserScan& scan) {


	scan.header.frame_id = grid.GetFrame();
	scan.header.stamp	 = ros::Time::now();
	scan.ranges			 = grid.GetGrid();
    scan.angle_min		 = grid.GetAngleMin();
    scan.angle_max		 = grid.GetAngleMax();
    scan.angle_increment = grid.GetAngleIncrement();
	scan.range_min		 = grid.GetRangeMin();
	scan.range_max		 = grid.GetRangeMax();

	return true;
}


	}
}

#endif
