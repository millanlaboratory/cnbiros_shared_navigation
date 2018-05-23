#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDDYNAMICS_CPP

#include "cnbiros_shared_navigation/SharedDynamics.hpp"

namespace cnbiros {
	namespace navigation {

SharedDynamics::SharedDynamics(void) : p_nh_("~") {

	this->n_rate_ = nullptr;

	// Dynamic reconfiguration
	this->n_dynreconfig_function_ = boost::bind(&SharedDynamics::reconfigure_callback, this, _1, _2);
	this->n_dynreconfig_server_.setCallback(this->n_dynreconfig_function_);

	// Configure node
	this->configure();

	// Initialize subscribers
    this->s_repellors_  = this->p_nh_.subscribe(this->t_repellors_, 1, &SharedDynamics::on_received_repellors, this);
    this->s_attractors_ = this->p_nh_.subscribe(this->t_attractors_, 1, &SharedDynamics::on_received_attractors, this);
    this->s_footprint_ = this->nh_.subscribe(this->t_footprint_, 1, &SharedDynamics::on_received_footprint, this);

	// Initialiaze publishers
	this->p_velocity_  = this->p_nh_.advertise<geometry_msgs::Twist>(this->t_velocity_, 1);
	this->p_visual_velocity_  = this->p_nh_.advertise<geometry_msgs::PoseStamped>(this->t_visual_velocity_, 1);

}

SharedDynamics::~SharedDynamics(void) {}

bool SharedDynamics::configure(void) {

	// Default topics
	this->t_repellors_		= "repellors";
	this->t_attractors_		= "attractors"; 
	this->t_footprint_		= "/move_base/local_costmap/footprint";
	this->t_velocity_		= "/cmd_vel";
	this->t_visual_velocity_ = "visual_velocity";

	this->robot_width_ = 1.0f;

	// Getting parameters

	this->p_nh_.param<bool>("enable_repellors",	 this->n_enable_repellors_, true);
	this->p_nh_.param<bool>("enable_attractors", this->n_enable_attractors_, true);
	this->p_nh_.param<float>("update_rate",	this->n_update_rate_, 10.0f);

	// Robot parameters
	this->p_nh_.param<std::string>("robot_base_frame", this->robot_base_frame_, "base_link");
	
	// Velocity Dynamics parameters 
	this->p_nh_.param<float>("angular_velocity_min", this->dyn_angular_velocity_min_, 0.17f);
	this->p_nh_.param<float>("angular_velocity_max", this->dyn_angular_velocity_max_, 0.3f);
	this->p_nh_.param<float>("angular_repellors_strength", this->dyn_angular_repellors_strength_, 5.0f);
	this->p_nh_.param<float>("angular_repellors_decay", this->dyn_angular_repellors_decay_, 1.0f);
	this->p_nh_.param<float>("angular_repellors_occupancy", this->dyn_angular_repellors_occupancy_, 5.0f);
	
	this->p_nh_.param<float>("linear_velocity_min", this->dyn_linear_velocity_min_, 0.08f);
	this->p_nh_.param<float>("linear_velocity_max", this->dyn_linear_velocity_max_, 0.250f);
	this->p_nh_.param<float>("linear_velocity_slope", this->dyn_linear_velocity_slope_, 0.03f);
	this->p_nh_.param<float>("linear_safe_distance", this->dyn_linear_safe_distance_, 1.2f);

	// Initialize boolean states
	this->is_data_available_ = false;

	// Initialize update rate
	this->init_update_rate(this->n_update_rate_);

	return true;
}

void SharedDynamics::Run(void) {


	while(this->nh_.ok()) {

		// Compute velocity
		this->MakeVelocity();


		// Publish velocity
		//this->velocity_.header.frame_id = this->robot_base_frame_;
		//this->velocity_.header.stamp	= ros::Time::now();
		this->p_velocity_.publish(this->velocity_);



		// Publish velocity as Pose
		this->visual_velocity_.header.frame_id = this->robot_base_frame_;
		this->visual_velocity_.header.stamp = ros::Time::now();
		this->visual_velocity_.pose.position.x = 0.0;
		this->visual_velocity_.pose.position.y = 0.0;
		this->visual_velocity_.pose.orientation = tf::createQuaternionMsgFromYaw(this->velocity_.angular.z);
		this->p_visual_velocity_.publish(this->visual_velocity_);
;

		ros::spinOnce();
		this->n_rate_->sleep();
	}
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

	// Compute linear velocity (repellors based)
	if(this->is_data_available_ == true)	
		vlinear = this->get_linear_velocity(this->pr_repellors_);


	printf("Linear velocity:  %f\n", vlinear);
	printf("Angular velocity: %f\n", vangular);

	this->velocity_.linear.x	= vlinear;
	//this->velocity_.linear.x	= 0.0;
	this->velocity_.linear.y	= 0.0;
	this->velocity_.linear.z	= 0.0;
	this->velocity_.angular.x	= 0.0;
	this->velocity_.angular.y	= 0.0;
	this->velocity_.angular.z	= -vangular;

}


float SharedDynamics::get_angular_velocity_repellors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
	float cdistance, cangle;
	float clambda, csigma;
	float cw;
	float w = 0.0f;

	// Iterate over the sectors
	for(it=sectors.Begin(); it!=sectors.End(); ++it) {
	
		// Discard empty sectors
		if(std::isinf((*it)) == true)
			continue;

		// Get current distance and angle
		cdistance	= sectors.GetRadius(it);
		cangle		= sectors.GetAngle(it);

		// Check if the repellor is inside the projection with respect to the
		// current wheelchair heading direction (angle=0.0). If it is outside,
		// then discard the repellor. Correct the current angle for standard
		// coordinates.
		if(this->is_projection_inside(cdistance, cangle+M_PI/2.0f, this->robot_width_) == false)
			continue;

		// Compute the contribution to the velocity
		clambda = this->dyn_angular_repellors_strength_*exp(-(cdistance/this->dyn_angular_repellors_decay_));
		csigma  = this->dyn_angular_repellors_occupancy_;
		cw = clambda*(-cangle)*std::exp(-(std::pow(cangle, 2))/2.0f*pow(csigma, 2));
	
		w += cw;
	}

	printf("Angular velocity: %f\n", w);

	return w;
}

float SharedDynamics::get_angular_velocity_attractors(ProximitySector& sectors) {
	return 0.0f;
}

float SharedDynamics::get_linear_velocity(ProximitySector& sectors) {

	ProximitySectorConstIt it;
	float cdistance, cangle;
	float min_obstacle_distance, min_obstacle_angle;
	float safe_obstacle_distance, min_linear_velocity, max_linear_velocity;
	float m;
	float y;

	min_obstacle_distance = 6.0f;
	min_obstacle_angle    = 0.0f;
	safe_obstacle_distance = this->dyn_linear_safe_distance_;
	min_linear_velocity    = this->dyn_linear_velocity_min_;
	max_linear_velocity    = this->dyn_linear_velocity_max_;
	m = this->dyn_linear_velocity_slope_;

	// Iterate over the sectors
	for(it=sectors.Begin(); it!=sectors.End(); ++it) {
		
		// Discard empty sectors
		if(std::isinf((*it)) == true)
			continue;
		
		// Get current distance and angle
		cdistance	= sectors.GetRadius(it);
		cangle		= sectors.GetAngle(it);

		// Check if the repellor is inside the projection with respect to the
		// current wheelchair heading direction (angle=0.0). If it is outside,
		// then discard the repellor. Correct the current angle for standard
		// coordinates.
		if(this->is_projection_inside(cdistance, cangle+M_PI/2.0f, this->robot_width_) == false)
			continue;

		if(cdistance < min_obstacle_distance) {
			min_obstacle_distance = cdistance;
			min_obstacle_angle	  = cangle;
		}
	}

	
	//if(min_obstacle_distance < safe_obstacle_distance)
	//	y = min_linear_velocity;
	//else
		y = m*(min_obstacle_distance - safe_obstacle_distance) + min_linear_velocity;


	if(y > max_linear_velocity)
		y = max_linear_velocity;
	
	//printf("Obstacle distance: %f\n", min_obstacle_distance);
	//printf("Safe distance: %f\n", safe_obstacle_distance);
	//printf("Min Linear velocity: %f\n", min_linear_velocity);

	return y;

}

bool SharedDynamics::is_projection_inside(float distance, float angle, float width) {

	if(std::fabs(distance*cos(angle)) <= width/2.0f)
		return true;
	else
		return false;
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

void SharedDynamics::on_received_footprint(const geometry_msgs::PolygonStamped& data) {

	this->get_robot_dimension(&(this->robot_lenght_), &(this->robot_width_), data.polygon);
	ROS_INFO_ONCE("Received robot footprint! Robot dimensions (length/width): (%f, %f) [m]",
					this->robot_lenght_, this->robot_width_);
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
	
	// Angular repellors occupancy
	if(this->update_if_different(config.angular_repellors_occupancy, this->dyn_angular_repellors_occupancy_))
		ROS_WARN("Updated angular repellors occupancy to %f", this->dyn_angular_repellors_occupancy_);
	
	// Linear minimum velocity
	if(this->update_if_different(config.linear_velocity_min, this->dyn_linear_velocity_min_))
		ROS_WARN("Updated linear velocity minimum to %f", this->dyn_linear_velocity_min_);
	
	// Linear maximum velocity
	if(this->update_if_different(config.linear_velocity_max, this->dyn_linear_velocity_max_))
		ROS_WARN("Updated linear velocity maximum to %f", this->dyn_linear_velocity_max_);
	
	// Linear slope velocity
	if(this->update_if_different(config.linear_velocity_slope, this->dyn_linear_velocity_slope_))
		ROS_WARN("Updated linear velocity slope to %f", this->dyn_linear_velocity_slope_);
	
	// Linear safe distance
	if(this->update_if_different(config.linear_safe_distance, this->dyn_linear_safe_distance_))
		ROS_WARN("Updated linear safe distance to %f", this->dyn_linear_safe_distance_);
}

void SharedDynamics::get_robot_dimension(float* lenght, float* width, geometry_msgs::Polygon polygon) {

	std::vector<float> x;
	std::vector<float> y;

	for(auto it=polygon.points.begin(); it!=polygon.points.end(); ++it) {
		x.push_back((*it).x);
		y.push_back((*it).y);
	}

	auto xres = std::minmax_element(x.begin(), x.end());
	auto yres = std::minmax_element(y.begin(), y.end());

	*lenght = std::fabs(*xres.first) + std::fabs(*xres.second);
	*width  = std::fabs(*yres.first) + std::fabs(*yres.second);
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

	}
}



#endif
