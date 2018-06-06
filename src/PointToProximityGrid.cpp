#ifndef CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYGRID_CPP
#define CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYGRID_CPP

#include "cnbiros_shared_navigation/PointToProximityGrid.hpp"

namespace cnbiros {
    namespace navigation {

PointToProximityGrid::PointToProximityGrid(void) : private_nh_("~") {

	// Configure node
	this->configure();
	
	// Initialize dynamic reconfiguration server
	this->f_ = boost::bind(&PointToProximityGrid::on_dynamic_reconfiguration, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);

    // Initialize subscriber and publisher
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::ProximityGridMsg>(this->ptopic_, 1);
	this->sub_ = this->nh_.subscribe(this->stopic_, 1, &PointToProximityGrid::on_received_point, this);
}

PointToProximityGrid::~PointToProximityGrid(void) {
	if(this->rate_ != nullptr)
		delete this->rate_;
}

bool PointToProximityGrid::configure(void) {

	float angle_min, angle_max, angle_inc, range_min, range_max, publish_frequency;
	std::string frame_id;

	this->rate_ = nullptr;

	// Getting parameters
	this->private_nh_.param<std::string>("source", this->stopic_, "/bci_command");
	this->private_nh_.param<std::string>("grid", this->ptopic_, "/proximity_grid");
	this->private_nh_.param<std::string>("frame_id", frame_id, "hokuyo_link");
	this->private_nh_.param<float>("angle_min",  angle_min, -120.0f);
	this->private_nh_.param<float>("angle_max",  angle_max, 120.0f);
	this->private_nh_.param<float>("angle_inc", angle_inc, 9.0f);
	this->private_nh_.param<float>("range_min",  range_min, 0.0f);
	this->private_nh_.param<float>("range_max",  range_max, 6.0f);
	this->private_nh_.param<float>("publish_frequency", this->publish_frequency_, 20.0);
   
	// Dump parameters
	//ROS_INFO("[PointToGrid] subscribed topic: %s", this->stopic_.c_str());
	ROS_INFO("[PointToGrid] advertised topic: %s", this->ptopic_.c_str());
	ROS_INFO("[PointToGrid] frame_id: %s", frame_id.c_str());
	ROS_INFO("[PointToGrid] minimum angle: %3.2f [deg]", angle_min);
	ROS_INFO("[PointToGrid] maximum angle: %3.2f [deg]", angle_max);
	ROS_INFO("[PointToGrid] angle increment: %3.2f [deg]", angle_inc);
	ROS_INFO("[PointToGrid] minimum range: %3.2f [m]", range_min);
	ROS_INFO("[PointToGrid] maximum range: %3.2f [m]", range_max);
	ROS_INFO("[PointToGrid] publish frequency: %3.2f", this->publish_frequency_);
	
	// Initialize rate
	this->init_update_rate(this->publish_frequency_); 
	
	// Instanciate sector vector
	this->grid_.SetAngleLimits(this->deg2rad(angle_min), this->deg2rad(angle_max)); 
	this->grid_.SetAngleIncrement(this->deg2rad(angle_inc));
	this->grid_.SetFrame(frame_id);
	this->grid_.SetRangeLimits(range_min, range_max);

	return true;
}

void PointToProximityGrid::Run(void) {

	cnbiros_shared_navigation::ProximityGridMsg		grid_msg;	

	while(this->nh_.ok()) {

		//ProximityGridConverter::ToMessage(this->grid_, grid_msg);
		//this->pub_.publish(grid_msg);
		
		ros::spinOnce();
		this->rate_->sleep();
	}
}

void PointToProximityGrid::on_dynamic_reconfiguration(cnbiros_shared_navigation::PointGridConfig &config, uint32_t level) {

	float angle_min, angle_max, angle_inc, range_min, range_max;

	angle_min   = this->grid_.GetAngleMin();
	angle_max   = this->grid_.GetAngleMax();
	angle_inc   = this->grid_.GetAngleIncrement();
	range_min   = this->grid_.GetRangeMin();
	range_max   = this->grid_.GetRangeMax();

	if(this->update_if_different(this->deg2rad(config.angle_min), angle_min)) {
		this->grid_.SetAngleLimits(angle_min, angle_max);
		ROS_WARN("[PointToGrid] Updated grid limits to (%3.2f, %3.2f) [deg]", 
				 this->rad2deg(angle_min), this->rad2deg(angle_max));
	}
	
	if(this->update_if_different(this->deg2rad(config.angle_max), angle_max)) {
		this->grid_.SetAngleLimits(angle_min, angle_max);
		ROS_WARN("[PointToGrid] Updated grid angle limits to (%3.2f, %3.2f) [deg]", 
				 this->rad2deg(angle_min), this->rad2deg(angle_max));
	}
	
	if(this->update_if_different(this->deg2rad(config.angle_inc), angle_inc)) {
		this->grid_.SetAngleIncrement(angle_inc);
		ROS_WARN("[PointToGrid] Updated grid angle increment to %3.2f [deg]", 
				 this->rad2deg(angle_inc));
	}
	
	if(this->update_if_different(config.range_min, range_min)) {
		this->grid_.SetRangeLimits(range_min, range_max);
		ROS_WARN("[PointToGrid] Updated grid range limits (minimum) to (%3.2f, %3.2f) [m]", 
				 range_min, range_max);
	}
	
	if(this->update_if_different(config.range_max, range_max)) {
		this->grid_.SetRangeLimits(range_min, range_max);
		ROS_WARN("[PointToGrid] Updated grid range limits (maximum) to (%3.2f, %3.2f) [m]", 
				 range_min, range_max);
	}
	
	if(this->update_if_different(config.publish_frequency, this->publish_frequency_)) {
		this->init_update_rate(this->publish_frequency_);
		ROS_WARN("[PointToGrid] Updated publish frequency to %f [Hz]", 
				 this->publish_frequency_);
	}
}

/******** Callback on received point ***********/
void PointToProximityGrid::on_received_point(const geometry_msgs::PointStamped& msg) {


	cnbiros_shared_navigation::ProximityGridMsg		grid_msg;	
	
	// Update the sector with LaserScan message
	if(ProximityGridConverter::FromPoint(msg, this->grid_, &(this->listener_)) == false) {
		ROS_ERROR("[PointToGrid] Cannot importing the incoming message into ProximityGrid");
	} 

	ProximityGridConverter::ToMessage(this->grid_, grid_msg);
	this->pub_.publish(grid_msg);
}

float PointToProximityGrid::rad2deg(float angle) {
	return angle*180.0f/M_PI;
}

float PointToProximityGrid::deg2rad(float angle) {
	return angle*M_PI/180.0f;
}

bool PointToProximityGrid::update_if_different(const float& first, float& second, float epsilon) {

	bool is_different = false;
	if(std::fabs(first - second) >= epsilon) {
		second = first;
		is_different = true;
	}

	return is_different;
}

void PointToProximityGrid::init_update_rate(float rate) {
	if(this->rate_ != nullptr)
		delete this->rate_;

	this->rate_ = new ros::Rate(rate);
}

    }
}

#endif
