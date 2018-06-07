#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDDYNAMICS_CPP

#include "cnbiros_shared_navigation/SharedDynamics.hpp"

namespace cnbiros {
	namespace navigation {

SharedDynamics::SharedDynamics(void) : p_nh_("~") {

	this->n_rate_ = nullptr;
	
	// Configure node
	this->configure();

	// Dynamic reconfiguration
	this->n_dynreconfig_function_ = boost::bind(&SharedDynamics::reconfigure_callback, this, _1, _2);
	this->n_dynreconfig_server_.setCallback(this->n_dynreconfig_function_);

	// Initialize subscribers
    this->s_repellors_  = this->p_nh_.subscribe(this->t_repellors_, 1, &SharedDynamics::on_received_repellors, this);
    this->s_attractors_ = this->p_nh_.subscribe(this->t_attractors_, 1, &SharedDynamics::on_received_attractors, this);
	// Initialiaze publishers
	this->p_velocity_	   = this->p_nh_.advertise<geometry_msgs::Twist>(this->t_velocity_, 1);

	// Initialize services
	this->srv_enable_ = this->p_nh_.advertiseService("navigation_enable", &SharedDynamics::on_requested_enable, this);
	this->srv_disable_ = this->p_nh_.advertiseService("navigation_disable", &SharedDynamics::on_requested_disable, this);
	this->srv_start_ = this->p_nh_.advertiseService("navigation_start", &SharedDynamics::on_requested_start, this);
	this->srv_stop_  = this->p_nh_.advertiseService("navigation_stop", &SharedDynamics::on_requested_stop, this);
}

SharedDynamics::~SharedDynamics(void) {
	if(this->n_rate_ != nullptr)
		delete this->n_rate_;
}

bool SharedDynamics::configure(void) {

	// Default topics
	this->t_repellors_	= "repellors";
	this->t_attractors_	= "attractors"; 
	this->t_velocity_	= "/cmd_vel";

	this->size_right_	= 0.375f;
	this->size_left_	= 0.375f;

	// Getting parameters
	this->p_nh_.param<bool>("enable_repellors",	 this->n_enable_repellors_, true);
	this->p_nh_.param<bool>("enable_attractors", this->n_enable_attractors_, true);
	this->p_nh_.param<float>("update_rate",	this->n_update_rate_, 10.0f);
	this->p_nh_.param<float>("publish_frequency",	this->publish_frequency_, 10.0f);

	// Robot parameters
	this->p_nh_.param<std::string>("base_frame", this->base_frame_, "base_link");
	
	// Velocity Dynamics parameters 
	this->p_nh_.param<float>("safe_distance_front", this->safe_distance_front_, 0.1f);
	this->p_nh_.param<float>("safe_distance_lateral", this->safe_distance_lateral_, 0.0f);
	
	this->p_nh_.param<float>("angular_velocity_min", this->dyn_angular_velocity_min_, 0.0f);
	this->p_nh_.param<float>("angular_velocity_max", this->dyn_angular_velocity_max_, 0.3f);

	this->p_nh_.param<float>("linear_velocity_min", this->dyn_linear_velocity_min_, 0.10f);
	this->p_nh_.param<float>("linear_velocity_max", this->dyn_linear_velocity_max_, 0.15f);
	
	this->p_nh_.param<float>("angular_repellors_strength", this->dyn_angular_repellors_strength_, 0.5f);
	this->p_nh_.param<float>("angular_repellors_decay", this->dyn_angular_repellors_decay_, 0.8f);
	this->p_nh_.param<float>("angular_attractors_strength", this->dyn_angular_attractors_strength_, 1.0f);
	this->p_nh_.param<float>("linear_velocity_decay", this->dyn_linear_velocity_decay_, 1.0f);
	this->p_nh_.param<float>("target_duration", this->target_duration_, 5.0f);

	this->target_ = 0.0f;

	// Initialize boolean states
	this->is_data_available_ = false;

	// Initialize boolean
	this->is_running_ = false;
	this->Enable();
	
	// Initialize update rate
	this->init_update_rate(this->n_update_rate_);

	// Create publish timer
	this->publish_timer_ = this->p_nh_.createTimer(ros::Duration(1.0f/this->publish_frequency_), &SharedDynamics::on_publish_velocity, this);

	// Create target timer
	this->target_timer_ = this->nh_.createTimer(ros::Duration(this->target_duration_), &SharedDynamics::on_target_elapsed, this);
	this->target_timer_.stop();

	return true;
}

void SharedDynamics::Run(void) {


	while(this->nh_.ok()) {

		if(this->IsEnabled() == true) {
			// Compute velocity
			this->MakeVelocity();
		}

		ros::spinOnce();
		this->n_rate_->sleep();
	}
}


void SharedDynamics::MakeVelocity(void) {
	
	float vangular;
	float vangular_r	= 0.0f;
	float vangular_a	= 0.0f;
	float vlinear		= 0.0f;
	float vangular_limited, vangular_scaled;

	// Compute orientation for repellors
	if(this->n_enable_repellors_ == true) {
		vangular_r  = this->get_angular_velocity_repellors(this->pr_repellors_);
		ROS_DEBUG_NAMED("velocity_repellors", "Repellors angular velocity: %f [deg/s]", rad2deg(vangular_r));
	}
	
	// Compute orientation for attractors
	if(this->n_enable_attractors_ == true) {
		vangular_a = this->get_angular_velocity_attractors(this->target_);
		ROS_DEBUG_NAMED("velocity_attractors", "Attractors angular velocity: %f [deg/s]", rad2deg(vangular_a));
	}

	vangular = vangular_a - vangular_r;

	// Limit the output velocity between -max and max
	vangular_limited = this->limit_range_value(vangular, 
											  -this->dyn_angular_velocity_max_,
											   this->dyn_angular_velocity_max_);

	// Scale the output velocity assuming that the absolute value of the input
	// is between [0 max] and the scaled range must be between [velocity_min
	// velocity_max]
	vangular_scaled  = this->scale_range_value(vangular_limited, 0.0f, 
										this->dyn_angular_velocity_max_,
									    this->dyn_angular_velocity_min_, 
										this->dyn_angular_velocity_max_);
	
	// Compute linear velocity (repellors based)
	if(this->is_data_available_ == true)
		vlinear = this->get_linear_velocity_repellors(this->pr_repellors_);


	// Fill the Twist message
	//this->velocity_.linear.x	= 0.0;
	this->velocity_.linear.x	= vlinear;
	this->velocity_.linear.y	= 0.0;
	this->velocity_.linear.z	= 0.0;
	this->velocity_.angular.x	= 0.0;
	this->velocity_.angular.y	= 0.0;
	this->velocity_.angular.z	= -vangular_scaled;
}

bool SharedDynamics::IsEnabled(void) {
	return this->is_enabled_;
}

void SharedDynamics::Disable(void) {
	this->is_enabled_ = false;
	this->Stop();
	ROS_WARN("[SharedDynamics] Node has been disabled");
}

void SharedDynamics::Enable(void) {
	this->is_enabled_ = true;
	ROS_WARN("[SharedDynamics] Node has been enabled");
}

bool SharedDynamics::IsRunning(void) {
	return this->is_running_;
}

void SharedDynamics::Start(void) {
	this->is_running_ = true;
	ROS_WARN("[SharedDynamics] Node has been started");
}

void SharedDynamics::Stop(void) {
	this->is_running_ = false;
	this->velocity_.linear.x	= 0.0f;
	this->velocity_.linear.y	= 0.0f;
	this->velocity_.linear.z	= 0.0f;
	this->velocity_.angular.x	= 0.0f;
	this->velocity_.angular.y	= 0.0f;
	this->velocity_.angular.z	= 0.0f;
	this->p_velocity_.publish(this->velocity_);
	ROS_WARN("[SharedDynamics] Node has been stopped");
}

float SharedDynamics::get_angular_velocity_repellors(ProximityGrid& data) {

	ProximityGridConstIt	it;
	float distance, angle;
	float clambda, csigma, cdtheta;
	float cw;
	float w = 0.0f;

	float robot_width			= this->size_right_ + this->size_left_;
	float safe_distance_front	= this->safe_distance_front_;
	float safe_distance_lateral	= this->safe_distance_lateral_;

	float hangle = 0.0;

	
	// Iterate over the sectors
	for(it=data.Begin(); it!=data.End(); ++it) {
	
		// Get current distance and angle
		distance	= data.GetSectorValue(it);
		angle		= data.GetSectorAngle(it);
		
		// Discard empty sectors
		if(std::isinf(distance) == true || std::isnan(distance) == true )
			continue;

		
		// Compute the contribution to the velocity
		clambda = this->dyn_angular_repellors_strength_*
				  exp(-( (distance - safe_distance_front)/this->dyn_angular_repellors_decay_));

		// If distance is smaller than safe distance, maximum value of lambda
		if( distance < (safe_distance_front) )
			clambda = this->dyn_angular_repellors_strength_;

		// Fixed sector angle
		cdtheta = data.GetAngleIncrement();

		// Compute current sigma
		csigma  = std::atan( std::tan(cdtheta/2.0f) 
				  + (robot_width + safe_distance_lateral)/(robot_width + safe_distance_lateral + distance ) );

		// Compute current angular velocity for this sector
		cw = clambda*(hangle-angle)*std::exp(-(std::pow(hangle-angle, 2))/(2.0f*pow(csigma, 2)));

		// Update the total angular velocity
		w += cw;
	}

											
	return w;
}

float SharedDynamics::get_angular_velocity_attractors(float angle) {
	
	//float forcelet;

	//forcelet = -tau*std::sin(phi - psi);
	float w;
	float hangle = 0.0f;

	w =  this->dyn_angular_attractors_strength_*std::sin(hangle - angle);

	//printf("angle attractor: %3.2f | angular velocity attractor: %3.2f\n", angle, w );
	return w;
}

float SharedDynamics::get_linear_velocity_repellors(ProximityGrid& data) {

	ProximityGridConstIt	it;
	float distance, angle;
	float min_angle, min_distance;
	float safe_distance_front;
	float robot_width;
	float sangle;
	float x;
	float y;

	x = 6.0f;
	robot_width		    = this->size_right_ + this->size_left_ + 2.0f*this->safe_distance_lateral_;
	safe_distance_front = this->safe_distance_front_;

	// Iterate over the sectors
	for(it=data.Begin(); it!=data.End(); ++it) {
	
		distance = data.GetSectorValue(it);
		angle	 = data.GetSectorAngle(it);
		
		// Discard empty sectors
		if(std::isinf(distance) == true || std::isnan(distance) == true )
			continue;
		
		// Check if the repellor is inside the projection with respect to the
		// current wheelchair heading direction (angle=0.0). If it is outside,
		// then discard the repellor. Correct the current angle for standard
		// coordinates.
		if(this->is_projection_inside(distance, angle+M_PI/2.0f, robot_width) == false)
			continue;

		// If distance is smaller then the previous one, select this distance
		if(distance < x) {
			 x = distance;
			 min_angle = angle;
		}
	}

	// If distance smaller the safe distance, then fixed negative velocity.
	// Otherwise compute the velocity according to exponential function
	y = -0.2f;
	if (x > safe_distance_front) {
		y = this->dyn_linear_velocity_max_*
		    (1.0f - std::exp(- (x - safe_distance_front)/this->dyn_linear_velocity_decay_ ));
	}

	return y;
}

bool SharedDynamics::is_projection_inside(float distance, float angle, float width) {

	float projection;

	projection = std::fabs(distance*cos(angle));

	if(projection > width/2.0f)
		return false;
		
	return true;
}

void SharedDynamics::on_publish_velocity(const ros::TimerEvent& event) {

	if (this->IsEnabled() == true && this->IsRunning() == true) {
		this->p_velocity_.publish(this->velocity_);

		ROS_DEBUG_NAMED("velocity", "Published velocity: v=%3.2f [m/s], o=%3.2f [deg/s]", 
					this->velocity_.linear.x, this->rad2deg(this->velocity_.angular.z));
	}
}

void SharedDynamics::on_target_elapsed(const ros::TimerEvent& event) {

	this->target_ = 0.0f;
	this->target_timer_.stop();
	ROS_INFO("Target duration elapsed (%3.2f s)", this->target_duration_);
}

float SharedDynamics::scale_range_value(float x, float minx, float maxx, float miny, float maxy) {

	float	absx;
	float	signx = 1.0f;
	float	y;

	if(x < 0)
		signx = -signx;

	absx = std::fabs(x);

	y = signx*((maxy - miny)*( (absx - minx) / (maxx - minx) ) + miny);

	return y;
}

float SharedDynamics::limit_range_value(float x, float minx, float maxx) {

	float lx;

	lx = x;
	if(lx > maxx) 
		lx = maxx;
	else if (lx < minx)
		lx = minx;
	
	return lx;
}

void SharedDynamics::on_received_repellors(const cnbiros_shared_navigation::ProximityGridMsg& data) {

	// Set true the availability of repellors data (used for first iteration)
	ROS_WARN_ONCE("First proximity grid messaged received! Repellors data available.");
	this->is_data_available_ = true;
	
	// Convert and store repellor data
	if(ProximityGridConverter::FromMessage(data, this->pr_repellors_) == false) {
		ROS_ERROR("Cannot convert repellor proximity grid message");
	}
}

void SharedDynamics::on_received_attractors(const cnbiros_shared_navigation::ProximityGridMsg& data) {

	ProximityGridConstIt it;
	ProximityGrid		 grid;
	float				 angle;

	angle = 0.0f;
	// Convert and store attractor data
	if(ProximityGridConverter::FromMessage(data, grid) == false) {
		ROS_ERROR("Cannot convert attractors proximity grid message");
	}

	for(it = grid.Begin(); it != grid.End(); ++it) {

		// Discard empty sectors
		if(std::isinf(grid.GetSectorValue(it)) == true || std::isnan(grid.GetSectorValue(it)) == true )
			continue;

		this->target_ = grid.GetSectorAngle(it) + grid.GetAngleIncrement()/2.0f;
		this->target_timer_.stop();
		this->target_timer_.start();
	}

}

void SharedDynamics::reconfigure_callback(cnbiros_shared_navigation::SharedDynamicsConfig &config, 
										uint32_t level) {

	// Angular minimum velocity
	if(this->update_if_different(config.angular_velocity_min, this->dyn_angular_velocity_min_))
		ROS_WARN("Updated angular velocity minimum to %f", this->dyn_angular_velocity_min_);
	
	// Angular maximum velocity
	if(this->update_if_different(config.angular_velocity_max, this->dyn_angular_velocity_max_))
		ROS_WARN("Updated angular velocity maximum to %f", this->dyn_angular_velocity_max_);
	
	// Angular repellors strength
	if(this->update_if_different(config.angular_repellors_strength, this->dyn_angular_repellors_strength_))
		ROS_WARN("Updated angular repellors strength to %f", this->dyn_angular_repellors_strength_);
	
	// Angular repellors decay
	if(this->update_if_different(config.angular_repellors_decay, this->dyn_angular_repellors_decay_))
		ROS_WARN("Updated angular repellors decay to %f", this->dyn_angular_repellors_decay_);
	
	// Angular attractors strength
	if(this->update_if_different(config.angular_attractors_strength, this->dyn_angular_attractors_strength_))
		ROS_WARN("Updated angular attractors strength to %f", this->dyn_angular_attractors_strength_);
	
	// Linear minimum velocity
	if(this->update_if_different(config.linear_velocity_min, this->dyn_linear_velocity_min_))
		ROS_WARN("Updated linear velocity minimum to %f", this->dyn_linear_velocity_min_);
	
	// Linear maximum velocity
	if(this->update_if_different(config.linear_velocity_max, this->dyn_linear_velocity_max_))
		ROS_WARN("Updated linear velocity maximum to %f", this->dyn_linear_velocity_max_);
	
	// Linear velocity decay
	if(this->update_if_different(config.linear_velocity_decay, this->dyn_linear_velocity_decay_))
		ROS_WARN("Updated linear velocity decay to %f", this->dyn_linear_velocity_decay_);
	
	// Linear safe distance
	if(this->update_if_different(config.safe_distance_front, this->safe_distance_front_))
		ROS_WARN("Updated robot safe front distance to %f", this->safe_distance_front_);
	
	// Linear safe distance
	if(this->update_if_different(config.safe_distance_lateral, this->safe_distance_lateral_))
		ROS_WARN("Updated robot safe lateral distance to %f", this->safe_distance_lateral_);
	
	// Update rate
	if(this->update_if_different(config.update_rate, this->n_update_rate_)) {
		ROS_WARN("Updated rate to %f", this->n_update_rate_);
		this->init_update_rate(this->n_update_rate_);
	}
	
	// Target duration
	if(this->update_if_different(config.target_duration, this->target_duration_)) {
		ROS_WARN("Updated target duration to %f", this->target_duration_);
	    this->target_timer_.setPeriod(ros::Duration(this->target_duration_), true);
	}
	
	// Publish frequency
	if(this->update_if_different(config.publish_frequency, this->publish_frequency_)) {
		ROS_WARN("Updated publish frequency to %f", this->publish_frequency_);
	    this->publish_timer_.setPeriod(ros::Duration(1.0f/this->publish_frequency_), true);
	}
}


float SharedDynamics::rad2deg(float radians) {
	return radians*180.0f/M_PI;
}


void SharedDynamics::init_update_rate(float rate) {
	if(this->n_rate_ != nullptr)
		delete this->n_rate_;

	this->n_rate_ = new ros::Rate(rate);
}


bool SharedDynamics::update_if_different(const float& first, float& second, float epsilon) {

	bool is_different = false;
	if(std::fabs(first - second) >= epsilon) {
		second = first;
		is_different = true;
	}

	return is_different;
}

bool SharedDynamics::on_requested_enable(std_srvs::Empty::Request& req, 
										 std_srvs::Empty::Response& res) {

	if(this->IsEnabled() == false) {
		this->Enable();
	}
	return true;
}

bool SharedDynamics::on_requested_disable(std_srvs::Empty::Request& req, 
										 std_srvs::Empty::Response& res) {

	if(this->IsEnabled() == true) {
		this->Disable();
	}
	return true;
}

bool SharedDynamics::on_requested_start(std_srvs::Empty::Request& req, 
										 std_srvs::Empty::Response& res) {

	if(this->IsRunning() == false) {
		this->Start();
	}
	return true;
}

bool SharedDynamics::on_requested_stop(std_srvs::Empty::Request& req, 
										 std_srvs::Empty::Response& res) {

	if(this->IsRunning() == true) {
		this->Stop();
	}
	return true;
}

//float SharedDynamics::get_angular_velocity_repellors(sensor_msgs::LaserScan& scan) {
//
//	float distance, angle, angle_inc;
//	float clambda, csigma, cdtheta;
//	float cw;
//	float w = 0.0f;
//	float w_limited;
//	float w_scaled;
//
//	float robot_front_size = this->robot_size_front_;		
//	float robot_width	   = this->robot_size_right_ + this->robot_size_left_;
//	float safe_distance    = this->robot_safe_distance_;
//
//	float hangle = 0.0;
//
//	
//	// Iterate over the sectors
//	angle	 = scan.angle_min;
//	angle_inc = scan.angle_increment;
//	for(auto it=scan.ranges.begin(); it!=scan.ranges.end(); ++it) {
//	
//		// Get current distance and angle
//		distance	= (*it);
//		angle		+= angle_inc;
//		
//		// Discard empty sectors
//		if(std::isinf(distance) == true || std::isnan(distance) == true )
//			continue;
//
//		// Discard distance below the minimum range
//		if(distance < scan.range_min)
//			continue;
//		
//		// Compute the contribution to the velocity
//		clambda = this->dyn_angular_repellors_strength_*
//				  exp(-( (distance - robot_front_size - safe_distance)/this->dyn_angular_repellors_decay_));
//
//		if( distance < (robot_front_size + safe_distance) )
//			clambda = this->dyn_angular_repellors_strength_;
//
//		cdtheta = std::asin( (robot_width + safe_distance)/(distance) );
//
//		if( distance < (robot_width + safe_distance) )
//			cdtheta = M_PI/2.0f;
//
//		cdtheta = angle_inc;
//		csigma  = std::atan( std::tan(cdtheta/2.0f) 
//				  + (robot_width + safe_distance)/(robot_width + safe_distance + distance ) );
//		cw = clambda*(hangle-angle)*std::exp(-(std::pow(hangle-angle, 2))/(2.0f*pow(csigma, 2)));
//
//		w += cw;
//	}
//
//	// Limit the output velocity between -max and max
//	w_limited = this->limit_range_value(w, -this->dyn_angular_velocity_max_,
//										    this->dyn_angular_velocity_max_);
//
//	// Scale the output velocity assuming that the absolute value of the input
//	// is between [0 max] and the scaled range must be between [velocity_min
//	// velocity_max]
//	w_scaled  = this->scale_range_value(w_limited, 0.0f, 
//										this->dyn_angular_velocity_max_,
//									    this->dyn_angular_velocity_min_, 
//										this->dyn_angular_velocity_max_);
//											
//	printf("Angular velocity: %f\n", w);
//	printf("Angular velocity limited: %f\n", w_limited);
//	printf("Angular velocity scaled:  %f\n", w_scaled);
//
//	return w_scaled;
//}
//
//float SharedDynamics::get_linear_velocity_repellors(sensor_msgs::LaserScan& data) {
//
//	float distance, angle, angle_inc;
//	float min_distance;
//	float robot_width;
//	float sel_angle;
//	float x;
//	float y;
//
//	x = 6.0f;
//	robot_width  = this->robot_size_right_ + this->robot_size_left_;
//	min_distance = this->robot_safe_distance_;
//
//	// Iterate over the sectors
//	angle	  = data.angle_min;
//	angle_inc = data.angle_increment;
//	for(auto it=data.ranges.begin(); it!=data.ranges.end(); ++it) {
//	
//		distance = (*it);
//		angle	+= angle_inc;
//		
//		// Discard empty sectors
//		if(std::isinf(distance) == true || std::isnan(distance) == true )
//			continue;
//		
//		// Discard distance below the minimum range
//		if(distance < data.range_min)
//			continue;
//		
//		// Check if the repellor is inside the projection with respect to the
//		// current wheelchair heading direction (angle=0.0). If it is outside,
//		// then discard the repellor. Correct the current angle for standard
//		// coordinates.
//		if(this->is_projection_inside(distance, angle+M_PI/2.0f-angle_inc, robot_width) == false)
//			continue;
//
//		if(distance < x) {
//			 x = distance;
//			 sel_angle = angle - angle_inc;
//		}
//	}
//
//	printf("closest distance: %f\n mindistance: %f\n minangle: %f\n", x, min_distance, sel_angle);
//	if (x < min_distance) {
//		y = -0.2f;
//	} else {
//		y = this->dyn_linear_velocity_max_*(1.0f - std::exp(- (x - min_distance)/this->dyn_linear_velocity_decay_ ));
//		//y = this->dyn_linear_velocity_max_;
//	}
//
//	//y = this->compute_linear_velocity(x, min_distance, max_distance, min_vel, max_vel, weight);
//
//	return y;
//}

//float SharedDynamics::get_angular_velocity_repellors(ProximitySector& sectors) {
//
//	ProximitySectorConstIt it;
//	float cdistance, cangle;
//	float clambda, csigma, cdtheta;
//	float cw;
//	float w = 0.0f;
//	float w_limited;
//	float w_scaled;
//
//	float cdpsi;
//	float obs_size	 = 0.05f;
//	float robot_size = 0.75f;
//	float robot_front_size = 0.93f;		// with respect to base_link
//	float safe_distance = 0.1f;
//	float robot_width = 0.75f;
//	float delta = this->dyn_angular_repellors_occupancy_;
//	float hangle = M_PI/2.0f;
//	float space_influence;
//
//	// Iterate over the sectors
//	for(it=sectors.Begin(); it!=sectors.End(); ++it) {
//	
//		// Discard empty sectors
//		if(std::isinf((*it)) == true)
//			continue;
//
//		// Get current distance and angle
//		cdistance	= sectors.GetRadius(it);
//		cangle		= sectors.GetAngle(it) + M_PI/2.0f;
//		
//		// Check if the repellor is inside the projection with respect to the
//		// current wheelchair heading direction (angle=0.0). If it is outside,
//		// then discard the repellor. Correct the current angle for standard
//		// coordinates.
//		//if(this->is_projection_inside(cdistance, cangle+M_PI/2.0f, this->robot_width_) == false)
//		//	continue;
//
//		// Compute the contribution to the velocity
//		clambda = this->dyn_angular_repellors_strength_*
//				  exp(-( (cdistance - robot_front_size - safe_distance)/this->dyn_angular_repellors_decay_));
//
//		if( cdistance < (robot_front_size + safe_distance) )
//			clambda = this->dyn_angular_repellors_strength_;
//
//		cdtheta = std::asin( (robot_width + safe_distance)/(cdistance) );
//
//		if( cdistance < (robot_width + safe_distance) )
//			cdtheta = M_PI/2.0f;
//
//		csigma  = std::atan( std::tan(cdtheta/2.0f) 
//				  + (robot_width + safe_distance)/(robot_width + safe_distance + cdistance ) );
//		cw = clambda*(hangle-cangle)*std::exp(-(std::pow(hangle-cangle, 2))/2.0f*pow(csigma, 2));
//
//		//space_influence = this->dyn_angular_repellors_decay_;
//
//
//		//cdpsi = this->get_subtended_angle(cdistance, obs_size, robot_size);
//		//cw = this->dyn_angular_repellors_strength_*this->get_angular_repellor_forcelet(hangle, cangle, cdpsi) * 
//		//	 this->get_angular_repellor_range(hangle, cangle, cdpsi, delta) *
//		//	 this->get_angular_repellor_decay(cdistance, obs_size, robot_size, space_influence);
//
//		//printf("cw=%f\n", cw);
//		w += cw;
//	}
//
//	// Limit the output velocity between -max and max
//	w_limited = this->limit_range_value(w, -this->dyn_angular_velocity_max_,
//										    this->dyn_angular_velocity_max_);
//
//	// Scale the output velocity assuming that the absolute value of the input
//	// is between [0 max] and the scaled range must be between [velocity_min
//	// velocity_max]
//	w_scaled  = this->scale_range_value(w_limited, 0.0f, this->dyn_angular_velocity_max_,
//									    this->dyn_angular_velocity_min_, this->dyn_angular_velocity_max_);
//											
//	printf("Angular velocity: %f\n", w);
//	printf("Angular velocity limited: %f\n", w_limited);
//	printf("Angular velocity scaled:  %f\n", w_scaled);
//
//	return w_scaled;
//}



//float SharedDynamics::get_angular_attractor_forcelet(float phi, float psi, float tau) {
//	
//	float forcelet;
//
//	forcelet = -tau*std::sin(phi - psi);
//
//	return forcelet;
//}

//float SharedDynamics::get_angular_repellor_forcelet(float phi, float psi, float dpsitot) {
//	float forcelet;
//
//	forcelet = (phi - psi)*(1.0f/dpsitot)*std::exp(1.0 - std::fabs( (phi - psi)/dpsitot) );
//
//	return forcelet;
//}

//float SharedDynamics::get_subtended_angle(float obs_distance, float obs_size, float robot_size) {
//
//	float dpsitot;
//	dpsitot = std::asin( (obs_size + robot_size) / (obs_distance + obs_size + robot_size) );
//
//	return dpsitot;
//}

//float SharedDynamics::get_angular_repellor_range(float phi, float psi, float dpsitot, float delta) {
//
//	float h1;
//	float range;
//
//	h1 = 4.0f/( std::cos(2.0f*dpsitot) - std::cos(2.0f*dpsitot + delta) );
//
//	range = 0.5f* ( std::tanh( h1*( std::cos(phi - psi) - std::cos(2.0f*dpsitot + delta) ) ) + 1.0f );
//
//	return  range;
//}

//float SharedDynamics::get_angular_repellor_decay(float obs_distance, float obs_size, float robot_size, float influence) {
//	
//	float decay;
//
//	decay = std::exp(- ( (obs_distance + obs_size + robot_size)/influence) );
//	return decay;
//}

//float SharedDynamics::get_angular_velocity_attractors(ProximitySector& sectors) {
//	return this->get_angular_attractor_forcelet(M_PI/2.0, M_PI/2.0, 0.1f);
//}


//void SharedDynamics::on_received_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {
//
//	// Set true the availability of repellors data (used for first iteration)
//	ROS_WARN_ONCE("First proximity sector messaged received! Repellors data available.");
//	this->is_data_available_ = true;
//	
//	// Convert and store repellor data
//	if(ProximitySectorConverter::FromMessage(data, this->pr_repellors_) == false) {
//		ROS_ERROR("Cannot convert repellor proximity sector message");
//	}
//}
//
//void SharedDynamics::on_received_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {
//
//	// Convert and store attractor data
//	if(ProximitySectorConverter::FromMessage(data, this->pr_attractors_) == false) {
//		ROS_ERROR("Cannot convert attractors proximity sector message");
//	}
//}


//void SharedDynamics::on_received_footprint(const geometry_msgs::PolygonStamped& data) {
//
//	this->get_robot_dimension(&(this->robot_lenght_), &(this->robot_width_), data.polygon);
//	ROS_INFO_ONCE("Received robot footprint! Robot dimensions (length/width): (%f, %f) [m]",
//					this->robot_lenght_, this->robot_width_);
//}

//void SharedDynamics::get_robot_dimension(float* lenght, float* width, geometry_msgs::Polygon polygon) {
//
//	//std::vector<float> x;
//	//std::vector<float> y;
//
//	//for(auto it=polygon.points.begin(); it!=polygon.points.end(); ++it) {
//	//	x.push_back((*it).x);
//	//	y.push_back((*it).y);
//	//}
//
//	//auto xres = std::minmax_element(x.begin(), x.end());
//	//auto yres = std::minmax_element(y.begin(), y.end());
//
//	//*lenght = *xres.second - *xres.first;
//	//*width  = *yres.second - *yres.first;
//	*lenght = 1.6;
//	*width  = 0.75;
//}

	}
}



#endif
