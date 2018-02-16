#ifndef CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/CostMapToProximitySector.hpp"

namespace cnbiros {
    namespace navigation {

CostMapToProximitySector::CostMapToProximitySector(void) : private_nh_("~"), listener_(ros::Duration(10)) {

	// Configure node
	this->configure();

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 50, &CostMapToProximitySector::on_received_costmap, this);
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximitySector>(this->ptopic_, 1000);

	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&CostMapToProximitySector::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);
}

CostMapToProximitySector::~CostMapToProximitySector(void) {}

bool CostMapToProximitySector::configure(void) {

	// Getting parameters
	this->private_nh_.param<std::string>("costmap/frame_id", this->frame_id_, "base_link");
	this->private_nh_.param<std::string>("costmap/source", this->stopic_, "/move_base/local_costmap/costmap");
	this->private_nh_.param<std::string>("costmap/sectors", this->ptopic_, "/proximitysector");
	this->private_nh_.param<float>("costmap/threshold",	this->threshold_, 100.0f);
	this->private_nh_.param<float>("costmap/min_angle",  this->min_angle_, -M_PI/2.0f);
	this->private_nh_.param<float>("costmap/max_angle",  this->max_angle_, M_PI/2.0f);
	this->private_nh_.param<int>("costmap/nsectors", this->nsectors_, 41);
   
	// Dump parameters
	ROS_INFO("CostMapToProximitySector frame_id: %s", this->frame_id_.c_str());
	ROS_INFO("CostMapToProximitySector subscribed topic: %s", this->stopic_.c_str());
	ROS_INFO("CostMapToProximitySector advertised topic: %s", this->ptopic_.c_str());
	ROS_INFO("CostMapToProximitySector threshold: %f", this->threshold_);
	ROS_INFO("CostMapToProximitySector minimum angle: %f [deg]", this->min_angle_*180.0f/M_PI);
	ROS_INFO("CostMapToProximitySector maximum angle: %f [deg]", this->max_angle_*180.0f/M_PI);
	ROS_INFO("CostMapToProximitySector number of sectors: %d", this->nsectors_);
	
	// Initialize sector vector
    this->init_sectors();

	return true;
}

void CostMapToProximitySector::on_dynamic_reconfiguration(cnbiros_shared_navigation::CostMapSectorConfig &config, uint32_t level) {



	if(std::fabs(config.threshold - this->threshold_) > 0.00001f) {
		this->threshold_ = config.threshold;
		ROS_WARN("Updated costmap threshold to %f", this->threshold_);
	}
	
	if(std::fabs(config.min_angle - this->min_angle_) > 0.00001f) {
		this->min_angle_ = config.min_angle;
		this->init_sectors();
		ROS_WARN("Updated minimum sector angle to %f [deg]", this->min_angle_*180.0f/M_PI);
	}
	
	if(std::fabs(config.max_angle - this->max_angle_) > 0.00001f) {
		this->max_angle_ = config.max_angle;
		this->init_sectors();
		ROS_WARN("Updated maximum sector angle to %f [deg]", this->max_angle_*180.0f/M_PI);
	}
	
	if(std::fabs(config.num_sectors - this->nsectors_) > 0.00001f) {
		this->nsectors_ = config.num_sectors;
		this->init_sectors();
		ROS_WARN("Updated number of sector to %d", this->nsectors_);
	}
}


/******** Sectors related methods ***********/
void CostMapToProximitySector::init_sectors(void) {

	// Compute the angular step between each sector
    this->step_ = (this->max_angle_ - this->min_angle_)/(float)this->nsectors_;

    // Initialize sector vector
    this->sectors_.reserve(this->nsectors_);
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void CostMapToProximitySector::reset_sectors(void) {
    this->sectors_.clear();
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void CostMapToProximitySector::set_sectors(float angle, float radius) {

    unsigned int idsector;
    float cvalue;
   
    // Determin the current sector, given the angle
    idsector = std::floor(angle/this->step_);

    // Get the current value of the sector
    cvalue = this->sectors_.at(idsector);

    // Replace the value of the sector with the minimum between the current
    // value and the given radius
    this->sectors_.at(idsector) = std::min(cvalue, radius);
}

void CostMapToProximitySector::dump_sectors(void) {

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

void CostMapToProximitySector::debug_dump_sectors(void) {

    float sector_lower, sector_upper, sector_value;
	unsigned int i = 0;
	
	for(auto it=this->sectors_.begin(); it!=this->sectors_.end(); ++it) {
		sector_lower = (this->min_angle_ + i*this->step_)*180.0f/M_PI;
		sector_upper = sector_lower + this->step_*180.0f/M_PI;
		sector_value = (*it);
		ROS_DEBUG("ProximitySector %u [%2.1f<->%2.1f]: %f [m]", 
				  i, sector_lower, sector_upper, sector_value); 
		i++;
    }
}

/******** Callback on received occupancy grid ***********/
void CostMapToProximitySector::on_received_costmap(const nav_msgs::OccupancyGrid& msgin) {

    grid_map::GridMap			map;
    grid_map::Position			position;
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;
    cnbiros_shared_navigation::ProximitySector msgout;
    float value, radius, angle;
  
	// Convert Occupancy Grid to GridMap (for conveniency)
    if(grid_map::GridMapRosConverter::fromOccupancyGrid(msgin, "costmap", map) == false) {
		ROS_ERROR("Cannot convert occupancy grid to grid map");
		return;
    }

    // Resetting current sectors
    this->reset_sectors();

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

		const grid_map::Index index(*iterator);

		// Convert Index to Position
		if(map.getPosition(index, position) == false) {
		    ROS_ERROR("Cannot get position in grid map");
		    continue;
		}

		// Get current value
		value = map.at("costmap", index);

		// Check if the value is greater than threshold
		if(value >= this->threshold_) {

			// Convert into geometry point
		    map_point.header.frame_id = msgin.header.frame_id;
		    map_point.point.x = position(0);
		    map_point.point.y = position(1);
		    map_point.point.z = 0.0f;
		   
			// Apply transformation from map frame to base
		    try {
				this->listener_.transformPoint(this->frame_id_, map_point, base_point);
			
				// If in the front, compute the current angle and radius. Then,
				// update the sectors.
				if(base_point.point.x >= 0) {
				    angle  = atan2(base_point.point.x, -base_point.point.y);
				    radius = hypot(base_point.point.x, base_point.point.y); 
				    this->set_sectors(angle, radius);
				}
		    } catch(tf::TransformException& ex) {
				ROS_ERROR("Cannot transform map to base point: %s", ex.what());
		    }
		}
    }

	// Dump current values - For debug
	this->debug_dump_sectors();

	// Fill the ProximitySector messgae
    msgout.header.frame_id	= this->frame_id_;
    msgout.header.stamp		= ros::Time::now();
    msgout.values			= this->sectors_;
    msgout.min_angle		= this->min_angle_;
    msgout.max_angle		= this->max_angle_;
    msgout.nsectors			= this->nsectors_;
    msgout.step				= this->step_;

    this->pub_.publish(msgout);
}

    }
}

#endif
