#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDDYNAMICS_CPP

#include "cnbiros_shared_navigation/SharedDynamics.hpp"

namespace cnbiros {
	namespace navigation {

SharedDynamics::SharedDynamics(void) : p_nh_("~") {


	// Dynamic reconfiguration
	this->n_dynreconfig_function_ = boost::bind(&SharedDynamics::reconfigure_callback, this, _1, _2);
	this->n_dynreconfig_server_.setCallback(this->n_dynreconfig_function_);

	// Configure node
	this->configure();

	// Initialize subscribers
    this->s_repellors_  = this->nh_.subscribe(this->t_repellors_, 1, &SharedDynamics::on_received_repellors, this);
    this->s_attractors_ = this->nh_.subscribe(this->t_attractors_, 1, &SharedDynamics::on_received_attractors, this);


}

SharedDynamics::~SharedDynamics(void) {}

bool SharedDynamics::configure(void) {

	// Default topics
	this->t_repellors_		= "/repellors";
	this->t_attractors_		= "/attractors"; 

	// Getting parameters
	this->p_nh_.param<std::string>("robot_base_frame",		this->robot_base_frame_, "base_link");

	this->p_nh_.param<bool>("enable_repellors",	 this->n_enable_repellors_, true);
	this->p_nh_.param<bool>("enable_attractors", this->n_enable_attractors_, true);
	this->p_nh_.param<float>("angular_velocity_limit", this->dyn_angular_velocity_limit_, 0.3f);
	this->p_nh_.param<float>("update_rate",	this->n_update_rate_, 10.0f);

	// Initialize boolean states
	this->is_data_available_ = false;

	return true;
}

void SharedDynamics::Run(void) {
}


void SharedDynamics::MakeVelocity(void) {
	
	float vangular;
	float vangular_r	= 0.0f;
	float vangular_a	= 0.0f;
	float vlinear		= 0.0f;

	// Compute orientation for repellors
	if(this->n_enable_repellors_ == true) {
		vangular_r  = this->get_angular_velocity_repellors(this->pr_repellors_);
		ROS_DEBUG_NAMED("shareddynamics", "Repellor angular velocity: %f [deg/s]", rad2deg(vangular_r));
	}
	
	// Compute orientation for attractors
	if(this->n_enable_attractors_ == true) {
		vangular_a = this->get_angular_velocity_attractors(this->pr_attractors_);
		ROS_DEBUG_NAMED("shareddynamics", "Attractor angular velocity: %f [deg/s]", rad2deg(vangular_a));
	}

	vangular = vangular_a - vangular_r;

	// Limit angular velocity (optional to be tested)
	vangular = this->limit_angular_velocity(vangular, this->dyn_angular_velocity_limit_);

	// Compute linear velocity (repellors based)
	if(this->is_data_available_ == true)	
		vlinear = this->get_linear_velocity(this->pr_repellors_);


}


float SharedDynamics::get_angular_velocity_repellors(ProximitySector& sectors) {
}

float SharedDynamics::get_angular_velocity_attractors(ProximitySector& sectors) {
}

float SharedDynamics::get_linear_velocity(ProximitySector& sectors) {
}


float SharedDynamics::limit_angular_velocity(float w, float limit) {

	if( w > limit)
		return limit;
	else if( w < -limit)
		return -limit;
	else
		return w;
}

void SharedDynamics::on_received_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Set true the availability of repellors data (used for first iteration)
	ROS_WARN_ONCE("First proximity sector messaged received! Repellors data available.");
	this->is_data_available_ = true;
	
	// Convert and store repellor data
	if(ProximitySectorConverter::FromMessage(data, this->pr_repellors_) == false) {
		ROS_ERROR("Cannot convert repellor proximity sector message");
	}
}

void SharedDynamics::on_received_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Convert and store attractor data
	if(ProximitySectorConverter::FromMessage(data, this->pr_attractors_) == false) {
		ROS_ERROR("Cannot convert attractors proximity sector message");
	}
}

void SharedDynamics::reconfigure_callback(cnbiros_shared_navigation::SharedDynamicsConfig &config, 
										uint32_t level) {
}

float SharedDynamics::rad2deg(float radians) {
	return radians*180.0f/M_PI;
}

	}
}



#endif
