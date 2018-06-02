#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDVISUALIZER_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDVISUALIZER_CPP

#include "cnbiros_shared_navigation/ProximityGridVisualizer.hpp"

namespace cnbiros {
    namespace navigation {

ProximityGridVisualizer::ProximityGridVisualizer(void) : private_nh_("~") {

	// Configure node
	this->configure();

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 1, &ProximityGridVisualizer::on_received_grid, this);
    this->pub_ = this->nh_.advertise<sensor_msgs::LaserScan>(this->ptopic_, 1);
}

ProximityGridVisualizer::~ProximityGridVisualizer(void) {}

bool ProximityGridVisualizer::configure(void) {

	// Getting parameters
	this->private_nh_.param<std::string>("source", this->stopic_, "/proximity_grid");
	this->private_nh_.param<std::string>("visualization", this->ptopic_, "/visualization");
	this->private_nh_.param<float>("radius", this->radius_, 1.0f);

	return true;
}


/******** Callback on received grid ***********/
void ProximityGridVisualizer::on_received_grid(const cnbiros_shared_navigation::ProximityGridMsg& msg) {

	ProximityGrid			grid;
	sensor_msgs::LaserScan  scan;

	if(ProximityGridConverter::FromMessage(msg, grid) == false) {
		ROS_ERROR("Cannot importing the incoming message into ProximityGrid");
		return;
	} 
	
	if(ProximityGridConverter::ToLaserScan(grid, scan) == false) {
		ROS_ERROR("Cannot convert grid into LaserScan");
		return;
	} 

	// Only for visualization purposes (center the laser scan message)
	scan.angle_min = scan.angle_min + scan.angle_increment/2.0f;
	
	scan.intensities.reserve(scan.ranges.size());
	for(auto it = scan.ranges.begin(); it != scan.ranges.end(); ++it) {
		scan.intensities.push_back((*it));
		(*it) = this->radius_;
	}

	// Publish the message
	this->pub_.publish(scan);
}

    }
}

#endif
