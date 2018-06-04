#ifndef CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/LaserScanToProximitySector.hpp"

namespace cnbiros {
	namespace navigation {

LaserScanToProximitySector::LaserScanToProximitySector(void) : private_nh_("~") {

	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&LaserScanToProximitySector::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);
	
	// Configure node
	this->configure();

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 1, &LaserScanToProximitySector::on_received_laserscan, this);
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximitySectorMsg>(this->ptopic_, 1);
	this->vpub_ = this->nh_.advertise<sensor_msgs::LaserScan>(this->vtopic_, 1);
}

LaserScanToProximitySector::~LaserScanToProximitySector(void) {}

bool LaserScanToProximitySector::configure(void) {

	float minangle, maxangle;
	int nsectors;
	std::string frameid;
	
	// Getting parameters
	this->private_nh_.param<std::string>("scan", this->stopic_, "/hokuyo/scan");
	this->private_nh_.param<std::string>("sectors", this->ptopic_, "/proximitysector");
	this->private_nh_.param<std::string>("visualize_sectors", this->vtopic_, "/visualization/sectors");
	this->private_nh_.param<std::string>("frame_id", frameid, "base_link");
	this->private_nh_.param<float>("min_angle",  minangle, -M_PI/2.0f);
	this->private_nh_.param<float>("max_angle",  maxangle, M_PI/2.0f);
	this->private_nh_.param<int>("nsectors", nsectors, 9);
   
	// Dump parameters
	ROS_INFO("LaserScanToProximitySector subscribed topic: %s", this->stopic_.c_str());
	ROS_INFO("LaserScanToProximitySector advertised topic: %s", this->ptopic_.c_str());
	ROS_INFO("LaserScanToProximitySector visualization topic: %s", this->vtopic_.c_str());
	ROS_INFO("LaserScanToProximitySector frame_id: %s", frameid.c_str());
	ROS_INFO("LaserScanToProximitySector minimum angle: %f [deg]", minangle*180.0f/M_PI);
	ROS_INFO("LaserScanToProximitySector maximum angle: %f [deg]", maxangle*180.0f/M_PI);
	ROS_INFO("LaserScanToProximitySector number of sectors: %d", nsectors);
	
	// Instanciate sector vector
	this->sector_.SetResolution(nsectors);
	this->sector_.SetMinAngle(minangle); 
	this->sector_.SetMaxAngle(maxangle);
	this->sector_.SetFrameId(frameid);

	// Initialize visualization LaserScan message
	this->vsector_.header.frame_id = this->sector_.GetFrameId();
	this->vsector_.angle_min = this->sector_.GetMinAngle();
	this->vsector_.angle_max = this->sector_.GetMaxAngle();
	this->vsector_.angle_increment = this->sector_.GetStep();
	this->vsector_.range_min = 0.0f;
	this->vsector_.range_max = 5.0f;

	return true;

}

void LaserScanToProximitySector::on_dynamic_reconfiguration(cnbiros_shared_navigation::LaserScanSectorConfig &config, uint32_t level) {


	
	if(std::fabs(config.min_angle - this->sector_.GetMinAngle()) > 0.0000001f) {
		this->sector_.SetMinAngle(config.min_angle);
		this->vsector_.angle_min = this->sector_.GetMinAngle();
		ROS_WARN("Updated minimum sector angle to %f [deg]", (config.min_angle)*180.0f/M_PI);
	}
	
	if(std::fabs(config.max_angle - this->sector_.GetMaxAngle()) > 0.0000001f) {
		this->sector_.SetMaxAngle(config.max_angle);
		this->vsector_.angle_max = this->sector_.GetMaxAngle();
		ROS_WARN("Updated maximum sector angle to %f [deg]", (config.max_angle)*180.0f/M_PI);
	}
	
	if(std::fabs(config.num_sectors - this->sector_.GetResolution()) > 0.0000001f) {
		this->sector_.SetResolution(config.num_sectors);
		this->vsector_.angle_increment = this->sector_.GetStep();
		ROS_WARN("Updated number of sector to %d", config.num_sectors);
	}
}

/******** Callback on received LaserScan ***********/
void LaserScanToProximitySector::on_received_laserscan(const sensor_msgs::LaserScan& msg_in) {

	cnbiros_shared_navigation::ProximitySectorMsg msg_out;
    tf::TransformListener	listener;
	
	// Update the sector with point message
	if(ProximitySectorConverter::FromLaserScan(msg_in, &listener, this->sector_) == false) {
		ROS_ERROR("Cannot importing the incoming message into ProximitySector");
	} else {

		// Dump for debugging
		ProximitySectorConstIt	it;
		for(it=this->sector_.Begin(); it!=this->sector_.End(); ++it)
			ROS_DEBUG_NAMED("laserscantosector", "%s", this->sector_.ToString(it).c_str());

		// Convert sector to message
		ProximitySectorConverter::ToMessage(this->sector_, msg_out);

		// Convert in LaserScan message
		this->vsector_.ranges.clear();
		this->vsector_.intensities.clear();
		for(it=this->sector_.Begin(); it!=this->sector_.End(); ++it) {
			this->vsector_.ranges.push_back(1.0f);

			if(std::isinf(*it) == true) {
				this->vsector_.intensities.push_back(this->vsector_.range_max);
			} else {
				this->vsector_.intensities.push_back(*it);
			}

		}

		this->vsector_.header.stamp = ros::Time::now();
		this->vpub_.publish(this->vsector_);

		// Publish the message
		this->pub_.publish(msg_out);
	}
}


	}
}

#endif
