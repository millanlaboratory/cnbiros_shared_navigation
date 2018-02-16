#ifndef CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/PointToProximitySector.hpp"

namespace cnbiros {
    namespace navigation {

PointToProximitySector::PointToProximitySector(void) : private_nh_("~"), listener_(ros::Duration(10)) {

	// Configure node
	this->configure();

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 50, &PointToProximitySector::on_received_point, this);
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximitySectorMsg>(this->ptopic_, 1000);

	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&PointToProximitySector::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);
}

PointToProximitySector::~PointToProximitySector(void) {}

bool PointToProximitySector::configure(void) {

	// Getting parameters
	this->private_nh_.param<std::string>("point/frame_id", this->frame_id_, "base_link");
	this->private_nh_.param<std::string>("point/source", this->stopic_, "/point");
	this->private_nh_.param<std::string>("point/sectors", this->ptopic_, "/proximitysector");
	this->private_nh_.param<float>("point/min_angle",  this->min_angle_, -M_PI/2.0f);
	this->private_nh_.param<float>("point/max_angle",  this->max_angle_, M_PI/2.0f);
	this->private_nh_.param<int>("point/nsectors", this->nsectors_, 41);
   
	// Dump parameters
	ROS_INFO("PointToProximitySector frame_id: %s", this->frame_id_.c_str());
	ROS_INFO("PointToProximitySector subscribed topic: %s", this->stopic_.c_str());
	ROS_INFO("PointToProximitySector advertised topic: %s", this->ptopic_.c_str());
	ROS_INFO("PointToProximitySector minimum angle: %f [deg]", this->min_angle_*180.0f/M_PI);
	ROS_INFO("PointToProximitySector maximum angle: %f [deg]", this->max_angle_*180.0f/M_PI);
	ROS_INFO("PointToProximitySector number of sectors: %d", this->nsectors_);
	
	// Initialize sector vector
    this->init_sectors();

	return true;
}

void PointToProximitySector::on_dynamic_reconfiguration(cnbiros_shared_navigation::PointSectorConfig &config, uint32_t level) {

	
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
void PointToProximitySector::init_sectors(void) {

	// Compute the angular step between each sector
    this->step_ = (this->max_angle_ - this->min_angle_)/(float)this->nsectors_;

    // Initialize sector vector
    this->sectors_.reserve(this->nsectors_);
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void PointToProximitySector::reset_sectors(void) {
    this->sectors_.clear();
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void PointToProximitySector::set_sectors(float angle, float radius) {

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

void PointToProximitySector::dump_sectors(void) {

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

void PointToProximitySector::debug_dump_sectors(void) {

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

/******** Callback on received point ***********/
void PointToProximitySector::on_received_point(const geometry_msgs::PointStamped& msgin) {

}

    }
}

#endif
