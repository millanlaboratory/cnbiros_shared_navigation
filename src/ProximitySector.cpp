#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/ProximitySector.hpp"

namespace cnbiros {
    namespace navigation {

ProximitySector::ProximitySector(int nsectors, float minangle, 
								 float maxangle, std::string frameid) {

	// Store sectors parameters
	this->nsectors_	 = nsectors;
	this->min_angle_ = minangle;
	this->max_angle_ = maxangle;
	this->frame_id_  = frameid;

	// Initialize the inner vector
	this->init_sectors();
}

ProximitySector::~ProximitySector(void) {}

void ProximitySector::SetResolution(int nsectors) {
	
	// Store sector parameters
	this->nsectors_ = nsectors;
	
	// Initialize the inner vector
	this->init_sectors();
}

void ProximitySector::SetRange(float minangle, float maxangle) {

	// Store sector parameters
	this->min_angle_ = minangle;
	this->max_angle_ = maxangle;
	
	// Initialize the inner vector
	this->init_sectors();
}

void ProximitySector::SetFrameId(std::string frameid) {
	
	// Store sector parameters
	this->frame_id_ = frameid;
}

void ProximitySector::Reset(void) {

	// Reset sectors
	this->reset_sectors();
}

void ProximitySector::SetByPolar(float angle, float radius) {

	// Set sectors
	this->set_sectors(angle, radius);
}

void ProximitySector::SetByCartesian(float x, float y) {
	
	float angle, radius;

	// Cartesian to Polar conversion
	angle  = atan2(x, y);
	radius = hypot(x, y); 

	// Set sectors
	this->set_sectors(angle, radius);
}

void ProximitySector::Dump(void) {

	// Dump sectors
	this->dump_sectors();
}

float ProximitySector::GetRadius(const ProximitySectorIt& it) {
	return (*it);
}

float ProximitySector::GetAngle(const ProximitySectorIt& it) {
	
	auto index = std::distance(this->sectors_.begin(), it);
	return this->min_angle_ + this->step_*(index + 0.5);
}

bool ProximitySector::FromOccupancyGrid(const nav_msgs::OccupancyGrid& msg, float threshold) {
	
	grid_map::GridMap			map;
    grid_map::Position			position;
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;
    float value, radius, angle;
  
	// Convert Occupancy Grid to GridMap (for conveniency)
    if(grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "costmap", map) == false) {
		ROS_ERROR("Cannot convert occupancy grid to grid map");
		return false;
    }

    // Resetting current sectors
    this->Reset();

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

		const grid_map::Index index(*iterator);

		// Convert Index to Position
		if(map.getPosition(index, position) == false) {
		    ROS_ERROR("Cannot get position in grid map. Skipping..");
		    continue;
		}

		// Get current value
		value = map.at("costmap", index);

		// Check if the value is greater than threshold
		if(value >= threshold) {

			// Convert into geometry point
		    map_point.header.frame_id = msg.header.frame_id;
		    map_point.point.x = position(0);
		    map_point.point.y = position(1);
		    map_point.point.z = 0.0f;
		   
			// Apply transformation from map frame to base
		    try {
				this->listener_.transformPoint(this->frame_id_, map_point, base_point);
			
				// If in the front, update the sectors.
				if(base_point.point.x >= 0) {
					this->SetByCartesian(base_point.point.x, -base_point.point.y);
				}
		    } catch(tf::TransformException& ex) {
				ROS_ERROR("Cannot transform map to base point: %s", ex.what());
				return false;
		    }
		}
    }

	return true;
}


bool ProximitySector::FromPoint(const geometry_msgs::PointStamped& msg) {

	geometry_msgs::PointStamped	sector_point;
	
	// Apply transformation from msg frame to sector frame
	try {
		this->listener_.transformPoint(this->frame_id_, msg, sector_point);
	
		// If in the front, update the sectors.
		if(sector_point.point.x >= 0) {
			this->SetByCartesian(sector_point.point.x, -sector_point.point.y);
		}
	} catch(tf::TransformException& ex) {
		ROS_ERROR("Cannot transform map to base point: %s", ex.what());
		return false;
	}
}

bool ProximitySector::FromMessage(const cnbiros_shared_navigation::ProximitySectorMsg& msg) {

	// Reset current sectors
	this->Reset();

	// Re-Initialize sectors according to the incoming message
	this->frame_id_  = msg.header.frame_id;
	this->nsectors_  = msg.nsectors;
	this->min_angle_ = msg.min_angle;
	this->max_angle_ = msg.max_angle;
	this->init_sectors();

	// Copy the sector values
	this->sectors_ = msg.values;
}

cnbiros_shared_navigation::ProximitySectorMsg ProximitySector::ToMessage(void) {

	cnbiros_shared_navigation::ProximitySectorMsg msg;

    msg.header.frame_id	= this->frame_id_;
    msg.header.stamp	= ros::Time::now();
    msg.values			= this->sectors_;
    msg.min_angle		= this->min_angle_;
    msg.max_angle		= this->max_angle_;
    msg.nsectors		= this->nsectors_;
    msg.step			= this->step_;
}


///******** Private methods to handle sector vector ********///
void ProximitySector::init_sectors(void) {

	// Compute the angular step between each sector
    this->step_ = (this->max_angle_ - this->min_angle_)/(float)this->nsectors_;

    // Initialize sector vector
    this->sectors_.reserve(this->nsectors_);
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void ProximitySector::reset_sectors(void) {
    this->sectors_.clear();
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void ProximitySector::set_sectors(float angle, float radius) {

    unsigned int idsector;
    float cvalue;
   
    // Determine the current sector, given the angle
    idsector = std::floor(angle/this->step_);

    // Get the current value of the sector
    cvalue = this->sectors_.at(idsector);

    // Replace the value of the sector with the minimum between the current
    // value and the given radius
    this->sectors_.at(idsector) = std::min(cvalue, radius);
}

void ProximitySector::dump_sectors(void) {

    float sector_lower, sector_upper, sector_value;
	unsigned int i = 0;
	
	for(auto it=this->sectors_.begin(); it!=this->sectors_.end(); ++it) {
		sector_lower = (this->min_angle_ + i*this->step_)*180.0f/M_PI;
		sector_upper = sector_lower + this->step_*180.0f/M_PI;
		sector_value = (*it);
		ROS_INFO("ProximitySector %u [%2.1f<->%2.1f]: %f [m]", 
				  i, sector_lower, sector_upper, sector_value); 
		i++;
    }
}

	}
}

#endif
