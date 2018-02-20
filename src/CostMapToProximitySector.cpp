#ifndef CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/CostMapToProximitySector.hpp"

namespace cnbiros {
    namespace navigation {

CostMapToProximitySector::CostMapToProximitySector(void) : private_nh_("~") {

	// Configure node
	this->configure();

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 50, &CostMapToProximitySector::on_received_costmap, this);
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximitySectorMsg>(this->ptopic_, 1000);

	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&CostMapToProximitySector::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);
}

CostMapToProximitySector::~CostMapToProximitySector(void) {}

bool CostMapToProximitySector::configure(void) {

	float minangle, maxangle;
	int nsectors;
	std::string frameid;
	
	// Getting parameters
	this->private_nh_.param<std::string>("costmap/source", this->stopic_, "/move_base/local_costmap/costmap");
	this->private_nh_.param<std::string>("costmap/sectors", this->ptopic_, "/proximitysector");
	this->private_nh_.param<float>("costmap/threshold",	this->threshold_, 100.0f);
	this->private_nh_.param<std::string>("costmap/frame_id", frameid, "base_link");
	this->private_nh_.param<float>("costmap/min_angle",  minangle, -M_PI/2.0f);
	this->private_nh_.param<float>("costmap/max_angle",  maxangle, M_PI/2.0f);
	this->private_nh_.param<int>("costmap/nsectors", nsectors, 41);
   
	// Dump parameters
	ROS_INFO("CostMapToProximitySector subscribed topic: %s", this->stopic_.c_str());
	ROS_INFO("CostMapToProximitySector advertised topic: %s", this->ptopic_.c_str());
	ROS_INFO("CostMapToProximitySector threshold: %f", this->threshold_);
	ROS_INFO("CostMapToProximitySector frame_id: %s", frameid.c_str());
	ROS_INFO("CostMapToProximitySector minimum angle: %f [deg]", minangle*180.0f/M_PI);
	ROS_INFO("CostMapToProximitySector maximum angle: %f [deg]", maxangle*180.0f/M_PI);
	ROS_INFO("CostMapToProximitySector number of sectors: %d", nsectors);
	
	// Instanciate sector vector
	this->sector_.SetResolution(nsectors);
	this->sector_.SetMinAngle(minangle); 
	this->sector_.SetMaxAngle(maxangle);
	this->sector_.SetFrameId(frameid);

	return true;
}

void CostMapToProximitySector::on_dynamic_reconfiguration(cnbiros_shared_navigation::CostMapSectorConfig &config, uint32_t level) {



	if(std::fabs(config.threshold - this->threshold_) > 0.0000001f) {
		this->threshold_ = config.threshold;
		ROS_WARN("Updated costmap threshold to %f", this->threshold_);
	}
	
	if(std::fabs(config.min_angle - this->sector_.GetMinAngle()) > 0.0000001f) {
		this->sector_.SetMinAngle(config.min_angle);
		ROS_WARN("Updated minimum sector angle to %f [deg]", (config.min_angle)*180.0f/M_PI);
	}
	
	if(std::fabs(config.max_angle - this->sector_.GetMaxAngle()) > 0.0000001f) {
		this->sector_.SetMaxAngle(config.max_angle);
		ROS_WARN("Updated maximum sector angle to %f [deg]", (config.max_angle)*180.0f/M_PI);
	}
	
	if(std::fabs(config.num_sectors - this->sector_.GetResolution()) > 0.0000001f) {
		this->sector_.SetResolution(config.num_sectors);
		ROS_WARN("Updated number of sector to %d", config.num_sectors);
	}
}


/******** Callback on received occupancy grid ***********/
void CostMapToProximitySector::on_received_costmap(const nav_msgs::OccupancyGrid& msg_in) {

	cnbiros_shared_navigation::ProximitySectorMsg msg_out;
    tf::TransformListener	listener;
	
	// Update the sector with point message
	if(ProximitySectorConverter::FromOccupancyGrid(msg_in, &listener, this->sector_, this->threshold_) == false) {
		ROS_ERROR("Cannot importing the incoming message into ProximitySector");
	} else {

		// Debug
		//this->sector_.Dump();

		// Convert sector to message
		ProximitySectorConverter::ToMessage(this->sector_, msg_out);

		// Publish the message
		this->pub_.publish(msg_out);
	}
}

    }
}

#endif
