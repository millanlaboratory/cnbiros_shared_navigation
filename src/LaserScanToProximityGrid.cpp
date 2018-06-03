#ifndef CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/LaserScanToProximityGrid.hpp"

namespace cnbiros {
	namespace navigation {

LaserScanToProximityGrid::LaserScanToProximityGrid(void) : private_nh_("~") {

	// Configure node
	this->configure();
	
	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&LaserScanToProximityGrid::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);

    // Initialize publisher
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximityGridMsg>(this->pub_topic_, 1);

	// Initialize subscribers from the provided list
	for(auto i=0; i<this->src_topic_.size(); i++) {
		this->sub_src_.push_back(this->nh_.subscribe<sensor_msgs::LaserScan>(this->src_topic_.at(i), 1, 
					  boost::bind( &LaserScanToProximityGrid::on_received_laserscan, 
					  this, _1, i) ));
	}
}

LaserScanToProximityGrid::~LaserScanToProximityGrid(void) {
	if(this->rate_ != nullptr)
		delete this->rate_;
}

bool LaserScanToProximityGrid::configure(void) {

	float angle_min, angle_max, angle_inc, range_min, range_max, publish_frequency;
	std::string frame_id;
	std::string src_topics;	

	this->rate_ = nullptr;
	
	// Getting parameters
	this->src_topic_ = {"/hokuyo/scan"};
	this->private_nh_.getParam("sources", this->src_topic_);
	this->private_nh_.param<std::string>("grid", this->pub_topic_, "/proximity_grid");
	
	this->private_nh_.param<std::string>("frame_id", frame_id, "hokuyo_link");
	this->private_nh_.param<float>("angle_min",  angle_min, -M_PI/2.0f);
	this->private_nh_.param<float>("angle_max",  angle_max, M_PI/2.0f);
	this->private_nh_.param<float>("angle_inc", angle_inc, M_PI/20.0f);
	this->private_nh_.param<float>("range_min",  range_min, 0.0f);
	this->private_nh_.param<float>("range_max",  range_max, 6.0f);
	this->private_nh_.param<float>("frequency", this->publish_frequency_, 20.0);
   
	// Dump parameters
	for(auto it=this->src_topic_.begin(); it != this->src_topic_.end(); ++it)
		src_topics += (*it) + " ";

	ROS_INFO("[ScanToGrid] subscribed topics: %s", src_topics.c_str());
	ROS_INFO("[ScanToGrid] advertised topic: %s", this->pub_topic_.c_str());
	ROS_INFO("[ScanToGrid] frame_id: %s", frame_id.c_str());
	ROS_INFO("[ScanToGrid] minimum angle: %3.2f [deg]", this->rad2deg(angle_min));
	ROS_INFO("[ScanToGrid] maximum angle: %3.2f [deg]", this->rad2deg(angle_max));
	ROS_INFO("[ScanToGrid] angle increment: %3.2f [deg]", this->rad2deg(angle_inc));
	ROS_INFO("[ScanToGrid] minimum range: %3.2f [m]", range_min);
	ROS_INFO("[ScanToGrid] maximum range: %3.2f [m]", range_max);
	ROS_INFO("[ScanToGrid] publish frequency: %3.2f", this->publish_frequency_);

	// Initialize rate
	this->init_update_rate(this->publish_frequency_); 

	// Initialize grid
	this->grid_.SetAngleLimits(angle_min, angle_max); 
	this->grid_.SetAngleIncrement(angle_inc);
	this->grid_.SetFrame(frame_id);
	this->grid_.SetRangeLimits(range_min, range_max);
	
	// Inititalize map sources
	for(auto i = 0; i < this->src_topic_.size(); i++)
		this->grid_src_.insert( std::pair<unsigned int, ProximityGrid>(i, ProximityGrid()) );

	return true;
}

void LaserScanToProximityGrid::Run(void) {

	cnbiros_shared_navigation::ProximityGridMsg		grid_msg;	

	while(this->nh_.ok()) {

		// Reset the current grid
		this->grid_.Reset();

		// Update the current grid with the available sources
		for(auto it = this->grid_src_.begin(); it != this->grid_src_.end(); ++it)
			this->grid_ = this->grid_ + it->second;
		
		// Convert it into message and publish it	
		ProximityGridConverter::ToMessage(this->grid_, grid_msg);
		this->pub_.publish(grid_msg);
		
		ros::spinOnce();
		this->rate_->sleep();
	}
}

void LaserScanToProximityGrid::on_dynamic_reconfiguration(cnbiros_shared_navigation::LaserScanGridConfig &config, uint32_t level) {

	float angle_min, angle_max, angle_inc, range_min, range_max;

	angle_min   = this->grid_.GetAngleMin();
	angle_max   = this->grid_.GetAngleMax();
	angle_inc   = this->grid_.GetAngleIncrement();
	range_min   = this->grid_.GetRangeMin();
	range_max   = this->grid_.GetRangeMax();

	if(this->update_if_different(config.angle_min, angle_min)) {
		this->grid_.SetAngleLimits(angle_min, angle_max);
		ROS_WARN("[ScanToGrid] Updated grid limits to (%3.2f, %3.2f) [deg]", 
				 this->rad2deg(angle_min), this->rad2deg(angle_max));
	}
	
	if(this->update_if_different(config.angle_max, angle_max)) {
		this->grid_.SetAngleLimits(angle_min, angle_max);
		ROS_WARN("[ScanToGrid] Updated grid limits to (%3.2f, %3.2f) [deg]", 
				 this->rad2deg(angle_min), this->rad2deg(angle_max));
	}
	
	if(this->update_if_different(config.angle_inc, angle_inc)) {
		this->grid_.SetAngleIncrement(angle_inc);
		ROS_WARN("[ScanToGrid] Updated grid angle increment to %3.2f [deg]", 
				 this->rad2deg(angle_inc));
	}
	
	if(this->update_if_different(config.range_min, range_min)) {
		this->grid_.SetRangeLimits(range_min, range_max);
		ROS_WARN("[ScanToGrid] Updated grid range limits to (%3.2f, %3.2f) [m]",
				 range_min, range_max);
	}
	
	if(this->update_if_different(config.range_max, range_max)) {
		this->grid_.SetRangeLimits(range_min, range_max);
		ROS_WARN("[ScanToGrid] Updated grid range limits to (%3.2f, %3.2f) [m]",
				 range_min, range_max);
	}
	
	if(this->update_if_different(config.publish_frequency, this->publish_frequency_)) {
		this->init_update_rate(this->publish_frequency_);
		ROS_WARN("[ScanToGrid] Updated publish frequency to %f [Hz]", 
				 this->publish_frequency_);
	}
}

/******** Callback on received LaserScan ***********/
void LaserScanToProximityGrid::on_received_laserscan(const sensor_msgs::LaserScanConstPtr& msg, unsigned int id) {

	// Initialize the temporary grid with the same values of the current grid
	ProximityGrid grid_new(this->grid_);

	// Update the sector with LaserScan message
	if(ProximityGridConverter::FromLaserScan(*msg, grid_new, &(this->listener_)) == false) {
		ROS_ERROR("[ScanToGrid] Cannot importing the incoming message into ProximitySector");
	} 

	// Store the source grid 
	this->grid_src_[id] = grid_new;
}

float LaserScanToProximityGrid::rad2deg(float angle) {
	return angle*180.0f/M_PI;
}

float LaserScanToProximityGrid::deg2rad(float angle) {
	return angle*M_PI/180.0f;
}

bool LaserScanToProximityGrid::update_if_different(const float& first, float& second, float epsilon) {

	bool is_different = false;
	if(std::fabs(first - second) >= epsilon) {
		second = first;
		is_different = true;
	}

	return is_different;
}

void LaserScanToProximityGrid::init_update_rate(float rate) {
	if(this->rate_ != nullptr)
		delete this->rate_;

	this->rate_ = new ros::Rate(rate);
}

	}
}

#endif
