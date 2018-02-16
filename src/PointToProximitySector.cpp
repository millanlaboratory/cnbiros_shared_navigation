#ifndef CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/PointToProximitySector.hpp"

namespace cnbiros {
    namespace navigation {

PointToProximitySector::PointToProximitySector(void) : private_nh_("~") {

	// Configure node
	this->configure();

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 50, &PointToProximitySector::on_received_point, this);
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximitySectorMsg>(this->ptopic_, 1000);

	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&PointToProximitySector::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);
}

PointToProximitySector::~PointToProximitySector(void) {
	delete this->sector_;
}

bool PointToProximitySector::configure(void) {

	float minangle, maxangle;
	int nsectors;
	std::string frameid;

	// Getting parameters
	this->private_nh_.param<std::string>("point/source", this->stopic_, "/point");
	this->private_nh_.param<std::string>("point/sectors", this->ptopic_, "/proximitysector");
	this->private_nh_.param<std::string>("point/frame_id", frameid, "base_link");
	this->private_nh_.param<float>("point/min_angle",  minangle, -M_PI/2.0f);
	this->private_nh_.param<float>("point/max_angle",  maxangle, M_PI/2.0f);
	this->private_nh_.param<int>("point/nsectors", nsectors, 41);
   
	// Dump parameters
	ROS_INFO("PointToProximitySector subscribed topic: %s", this->stopic_.c_str());
	ROS_INFO("PointToProximitySector advertised topic: %s", this->ptopic_.c_str());
	ROS_INFO("PointToProximitySector frame_id: %s", frameid.c_str());
	ROS_INFO("PointToProximitySector minimum angle: %f [deg]", minangle*180.0f/M_PI);
	ROS_INFO("PointToProximitySector maximum angle: %f [deg]", maxangle*180.0f/M_PI);
	ROS_INFO("PointToProximitySector number of sectors: %d", nsectors);
	
	// Instanciate sector vector
	this->sector_ = new ProximitySector(nsectors, minangle, maxangle, frameid);

	return true;
}

void PointToProximitySector::on_dynamic_reconfiguration(cnbiros_shared_navigation::PointSectorConfig &config, uint32_t level) {

	
	if(std::fabs(config.min_angle - this->sector_->GetMinAngle()) > 0.00001f) {
		this->sector_->SetMinAngle(config.min_angle);
		ROS_WARN("Updated minimum sector angle to %f [deg]", config.min_angle*180.0f/M_PI);
	}
	
	if(std::fabs(config.max_angle - this->sector_->GetMaxAngle()) > 0.00001f) {
		this->sector_->SetMaxAngle(config.max_angle);
		ROS_WARN("Updated maximum sector angle to %f [deg]", config.max_angle*180.0f/M_PI);
	}
	
	if(std::fabs(config.num_sectors - this->sector_->GetResolution()) > 0.00001f) {
		this->sector_->SetResolution(config.num_sectors);
		ROS_WARN("Updated number of sector to %d", config.num_sectors);
	}
}



/******** Callback on received point ***********/
void PointToProximitySector::on_received_point(const geometry_msgs::PointStamped& msg_in) {

	cnbiros_shared_navigation::ProximitySectorMsg msg_out;

	// Update the sector with point message
	if(this->sector_->FromPoint(msg_in) == false) {
		ROS_ERROR("Cannot importing the incoming message into ProximitySector");
	} else {

		// Debug
		this->sector_->Dump();

		// Convert sector to message
		msg_out = this->sector_->ToMessage();

		// Publish the message
		this->pub_.publish(msg_out);
	}
}

    }
}

#endif
