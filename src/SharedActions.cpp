#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_CPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_CPP

#include "cnbiros_shared_navigation/SharedActions.hpp"

namespace cnbiros {
    namespace navigation {

SharedActions::SharedActions(void) : p_nh_("~") {

    this->a_client_ = nullptr;
    
	// Initialize user command timer
	this->init_command_timer(1.0f);

	// Initialize update rate
	this->init_update_rate(1.0f);
	
	// Dynamic reconfiguration
	this->n_dynreconfig_function_ = boost::bind(&SharedActions::reconfigure_callback, this, _1, _2);
	this->n_dynreconfig_server_.setCallback(this->n_dynreconfig_function_);
	
	// Configure node
	this->configure();

	// Initialize subscribers
    this->s_repellors_  = this->nh_.subscribe(this->t_repellors_, 1, &SharedActions::on_received_repellors, this);
    this->s_attractors_ = this->nh_.subscribe(this->t_attractors_, 1, &SharedActions::on_received_attractors, this);

	// Initialize move base client
    this->a_client_ = new MoveBaseClient(this->a_server_name_, true);

	// Initialize start/stop service server
	this->srv_state_toggle_ = this->p_nh_.advertiseService("toggle_state", &SharedActions::on_requested_state_toggle, this);

	this->srv_set_goal_ = this->p_nh_.advertiseService("set_goal", &SharedActions::on_requested_set_goal, this);

	// Initializer external goal reset area
	this->p_reset_area_ = this->p_nh_.advertise<geometry_msgs::PolygonStamped>(this->t_reset_area_, 1);
	
	// Initialize odom subcriber
	this->s_odom_ = this->nh_.subscribe(this->t_odom_, 1, &SharedActions::on_received_odometry, this);
	
	// If is running (autostart)
	if(this->n_autostart_)
		this->Start(); 
}

SharedActions::~SharedActions(void) {
    if(this->a_client_ == nullptr)
	delete this->a_client_;
}


bool SharedActions::configure(void) {

	// Default topics
	this->t_repellors_			= "/repellors";
	this->t_attractors_			= "/attractors"; 
	this->t_odom_				= "/odom";
	this->t_reset_area_			= "/visualize_reset_area";

	// Getting parameters
	this->p_nh_.param<std::string>("action_server_name",	this->a_server_name_, "move_base");
	this->p_nh_.param<std::string>("robot_base_frame",		this->robot_base_frame_, "base_link");
	this->p_nh_.param<float>("robot_size", this->robot_size_, 0.52f);
	
	this->p_nh_.param<bool>("autostart",			this->n_autostart_, false);
	this->p_nh_.param<bool>("enable_repellors",		this->n_enable_repellors_, true);
	this->p_nh_.param<bool>("enable_attractors",	this->n_enable_attractors_, true);
	this->p_nh_.param<float>("update_rate",			this->n_update_rate_, 10.0f);
	this->p_nh_.param<float>("command_timeout",		this->n_command_timeout_, 8.0f);
	
	this->p_nh_.param<float>("repellors_strength",	this->sa_repellors_strength_, 0.7f);
	this->p_nh_.param<float>("repellors_decay",		this->sa_repellors_decay_, 1.2f);
	this->p_nh_.param<float>("repellors_occupancy", this->sa_repellors_occupancy_, 0.5f);
	//this->p_nh_.param<float>("distance_slope",		this->sa_distance_slope_, 1.3f);
	this->p_nh_.param<float>("distance_max",		this->sa_distance_max_, 2.0f);
	this->p_nh_.param<float>("obstacle_padding",	this->sa_obstacle_padding_, 0.9f);
	this->p_nh_.param<float>("backward_limit",	    this->sa_backward_limit_, 1.0f);
	//this->p_nh_.param<float>("distance_zero",		this->sa_distance_zero_, 1.5f);
	this->p_nh_.param<float>("reset_radius",		this->sa_reset_radius_, 1.8f);

	// Create reset area 
	this->m_reset_area_ = this->get_reset_area(this->sa_reset_radius_);	

	// Initialize to false availability of repellors_data
	this->is_data_available_ = false;

	// Initialize is running to false
	this->is_running_ = false;

	return true;
}

void SharedActions::reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, 
										uint32_t level) {


	if(this->update_if_different(config.repellors_strength, this->sa_repellors_strength_))
		ROS_WARN("Updated repellors strength to %f", this->sa_repellors_strength_);
	
	if(this->update_if_different(config.repellors_decay, this->sa_repellors_decay_))
		ROS_WARN("Updated repellors decay to %f", this->sa_repellors_decay_);
	
	if(this->update_if_different(config.repellors_occupancy, this->sa_repellors_occupancy_))
		ROS_WARN("Updated repellors occupancy to %f", this->sa_repellors_occupancy_);
	
	//if(this->update_if_different(config.distance_slope, this->sa_distance_slope_))
	//	ROS_WARN("Updated distance slope to %f", this->sa_distance_slope_);
	
	if(this->update_if_different(config.obstacle_padding, this->sa_obstacle_padding_))
		ROS_WARN("Updated obstacle padding to %f", this->sa_obstacle_padding_);
	
	if(this->update_if_different(config.distance_max, this->sa_distance_max_))
		ROS_WARN("Updated distance max to %f", this->sa_distance_max_);
	
	//if(this->update_if_different(config.distance_zero, this->sa_distance_zero_))
	// 	ROS_WARN("Updated distance zero to %f", this->sa_distance_zero_);
	
	if(this->update_if_different(config.backward_limit, this->sa_backward_limit_))
		ROS_WARN("Updated backward limit to %f", this->sa_backward_limit_);
	
	if(this->update_if_different(config.reset_radius, this->sa_reset_radius_)) {
		this->m_reset_area_ = this->get_reset_area(this->sa_reset_radius_);	
		ROS_WARN("Updated reset radius to %f", this->sa_reset_radius_);
	}
	
	if(this->update_if_different(config.command_timeout, this->n_command_timeout_)) {
		ROS_WARN("Updated command timeout to %f", this->n_command_timeout_);
	}
	
	if(this->update_if_different(config.update_rate, this->n_update_rate_)) {
		this->change_update_rate(this->n_update_rate_);	
		ROS_WARN("Updated node rate to %f", this->n_update_rate_);
	}

}

bool SharedActions::update_if_different(const float& first, float& second, float epsilon) {

	bool is_different = false;
	if(std::fabs(first - second) >= epsilon) {
		second = first;
		is_different = true;
	}

	return is_different;
}

bool SharedActions::IsRunning(void) {
	return this->is_running_;
}

SharedActions::State SharedActions::GetState(void) {
	return this->n_state_;
}

bool SharedActions::IsConnected(void) {
	return this->a_client_->isServerConnected();
}

void SharedActions::Start(void) {
	if(this->IsRunning() == false) {
		this->is_running_ = true;
		this->n_state_ = SharedActions::State::Running;
		ROS_WARN("SharedActions started");
	}
}

void SharedActions::Stop(void) {
	if(this->IsRunning() == true) {
		this->is_running_ = false;
		this->n_state_ = SharedActions::State::Stopped;
		ROS_WARN("SharedActions stopped");
	}
}

void SharedActions::Pause(void) {
	if(this->IsRunning() == true) {
		this->is_running_ = false;
		this->n_state_ = SharedActions::State::Paused;
		ROS_WARN("SharedActions paused");
	}
}

void SharedActions::Quit(void) {
	if(this->IsRunning() == true) {
		this->is_running_ = false;
		this->n_state_ = SharedActions::State::Quit;
		ROS_WARN("SharedActions quit");
	}
}

void SharedActions::Run(void) {
   
	actionlib::SimpleClientGoalState a_state(actionlib::SimpleClientGoalState::LOST);
	std_srvs::Empty emptymsg;

    while(this->nh_.ok()) {
		
		// Spin and sleep
		ros::spinOnce();
		this->n_rate_->sleep();

		// Wait for connection of the action server
		if(this->IsConnected() == false) {
			ROS_WARN_THROTTLE(5.0f, "Waiting for %s action server to connect", this->a_server_name_.c_str());
			continue;
		}
		ROS_WARN_ONCE("'%s' action server connected", this->a_server_name_.c_str());

		// Publish reset area visualization
		this->m_reset_area_.header.frame_id = this->robot_base_frame_;
		this->m_reset_area_.header.stamp    = ros::Time::now();
		this->p_reset_area_.publish(this->m_reset_area_);
		
		// Check if the node state is running
		if(this->IsRunning() != true) {
			ROS_WARN_THROTTLE(5, "SharedAction waiting to be started...");
			continue;
		}

		// Start the main loop
		
		switch(this->GetState()) {
			case SharedActions::State::Quit:
				break;
			case SharedActions::State::Stopped:
				break;
			case SharedActions::State::Paused:
				break;
			case SharedActions::State::Running:
			
				if(this->is_within_reset_area(this->sa_reset_radius_)) {
					//ROS_INFO("Inside goal area (%3.2f m).", this->sa_reset_radius_);
					//this->CancelGoal();
					this->MakeGoal();
					this->SendGoal();
				}
				
				if(this->a_client_->getState() == actionlib::SimpleClientGoalState::ABORTED) {
					ROS_WARN("Goal aborted");
					this->CancelGoal();
					this->MakeGoal();
					this->SendGoal();

				} else if(this->a_client_->getState() == actionlib::SimpleClientGoalState::LOST) {
					ROS_WARN("Goal lost.");
					this->CancelGoal();
					this->MakeGoal();
					this->SendGoal();
				} else {
				}

				break;
			default:
				break;
		}
    }
}

float SharedActions::rad2deg(float radians) {
	return radians*180.0f/M_PI;
}

float SharedActions::deg2rad(float degrees) {
	return degrees*M_PI/180.0f;
}

void SharedActions::MakeGoal(void) {

	float angle;
	float r_angle = 0.0f;
	float a_angle = 0.0f;
	float distance = 0.0f;
	geometry_msgs::PointStamped m_goal_base_frame;

	
	// Compute orientation for repellors
	if(this->n_enable_repellors_ == true) {
		r_angle  = this->get_angle_on_repellors(this->pr_repellors_);
		ROS_DEBUG_NAMED("sharedactions", "Map angle (heading): %f [deg]", rad2deg(r_angle));
	}
	
	// Compute orientation for attractors
	if(this->n_enable_attractors_ == true) {
		a_angle = this->get_angle_on_attractors(this->pr_attractors_);
		ROS_DEBUG_NAMED("sharedactions", "Usr angle (heading): %f [deg]", rad2deg(a_angle));
	}
	
	angle = r_angle + a_angle;

	// Limit the orientation to the front
	angle = this->get_angle_limits(angle, -M_PI/2.0f, M_PI/2.0f);	
	
	// Compute position for computed angle (if repellors_data_ is available)
	if(this->is_data_available_ == true) {
		distance = this->get_distance(this->pr_repellors_, angle);
	}

	// Rotate angle (standard coordinate frame conventions)	
	ROS_DEBUG_NAMED("goal_angle", "[SharedActions] - Goal angle (sectors): %3.2f", this->rad2deg(angle));
	angle = M_PI/2.0f + angle;
	ROS_DEBUG_NAMED("goal_angle", "[SharedActions] - Goal angle (standard frame): %3.2f", this->rad2deg(angle));

    // Make the goal given the computed angle and radius
	this->m_goal_.target_pose.header.frame_id		= this->robot_base_frame_;
    this->m_goal_.target_pose.pose.position.x		= distance*sin(angle);
    this->m_goal_.target_pose.pose.position.y		= -distance*cos(angle);
    this->m_goal_.target_pose.pose.orientation		= tf::createQuaternionMsgFromYaw(angle-M_PI/2.0);
    //this->m_goal_.target_pose.pose.orientation.x	= 0;
    //this->m_goal_.target_pose.pose.orientation.y	= 0;
    //this->m_goal_.target_pose.pose.orientation.z	= 0;
    //this->m_goal_.target_pose.pose.orientation.w	= 1;
    this->m_goal_.target_pose.header.stamp			= ros::Time::now();

	// Extract the current goal point from the goal message (in the robot frame)
	m_goal_base_frame.header = this->m_goal_.target_pose.header;
	m_goal_base_frame.point  = this->m_goal_.target_pose.pose.position;
	
	// Transform current goal point in odom frame
	try {
		this->n_listener_.waitForTransform("odom", this->robot_base_frame_, 
										   m_goal_base_frame.header.stamp, ros::Duration(10.0) );
		this->n_listener_.transformPoint("odom", m_goal_base_frame, this->m_goal_point_);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}
}


void SharedActions::SendGoal(void) {

	if(this->IsConnected() == false) {
    	ROS_WARN("'%s' action server is disconnected. Nothing to do.", this->a_server_name_.c_str());
		return;
	}
	
	/*
	this->a_client_->sendGoal(this->m_goal_, 
							   MoveBaseClient::SimpleDoneCallback(), 
							   MoveBaseClient::SimpleActiveCallback(), 
							   boost::bind(&SharedActions::feedbackCb, this, _1));
							   */
	this->a_client_->sendGoal(this->m_goal_);

	ROS_DEBUG_NAMED("goal_position", "Sent new goal at (%4.2f, %4.2f) [m]", 
					this->m_goal_.target_pose.pose.position.x, this->m_goal_.target_pose.pose.position.y);
}

void SharedActions::CancelGoal(void) {

	if(this->IsConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->a_server_name_.c_str());
		return;
	}
	
	this->a_client_->cancelGoal();
}


bool SharedActions::is_projection_inside(float distance, float angle, float radius) {

	float projection;
	bool result = false;

	projection = std::fabs(distance*cos(angle)); 

	if(projection <= 2.0f*radius)
		result = true;

	return result;
}

float SharedActions::get_angle_on_repellors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
    float sector_step;
    float cdistance, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

	//float robotsize = 2.0f*(this->robot_radius_);

    sector_step	= sectors.GetStep();

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		//cdistance = sectors.GetRadius(it);
		cdistance = sectors.GetRadius(it);
		cangle  = sectors.GetAngle(it);

		// Check if the repellors is inside the robot radius. If not, skip it.
		//if(this->is_projection_inside(cdistance, cangle + M_PI/2.0f, this->robot_radius_) == false)
		//	continue;

		clambda = this->sa_repellors_strength_*exp(-(cdistance/this->sa_repellors_decay_));
		csigma  = std::atan(std::tan(sector_step/2.0f) + this->robot_size_/(this->robot_size_ + cdistance)) 
				  + this->sa_repellors_occupancy_;
		
		w += clambda*(hangle-cangle)*std::exp(-(std::pow(hangle - cangle, 2))/(2.0f*pow(csigma, 2)));
    }

    return w;
}

float SharedActions::get_angle_on_attractors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
    float w = 0.0f;

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		w  += sectors.GetAngle(it);
    }
    return w;
}

float SharedActions::get_angle_limits(float angle, float minangle, float maxangle) {

	float langle;

	langle = angle;
	if (angle < minangle) {
		langle = minangle;
	} else if(angle > maxangle) {
		langle = maxangle;
	}

	return langle;
}


float SharedActions::get_distance(ProximitySector& sectors, float wescape) {

	ProximitySectorConstIt it;
	float obstacle_distance;
	float distance;
	float m, b;
	float x1 = 1.0f;
	float y1 = x1 - this->sa_obstacle_padding_;
	float x2 = 2.0f;
	float y2 = x2 - this->sa_obstacle_padding_;

	try {
		obstacle_distance = sectors.At(wescape);
	} catch (std::runtime_error e) {
		ROS_ERROR("Error: %s", e.what());
	}

	if(std::isinf(obstacle_distance)) {
		distance = this->sa_distance_max_;
	} else if (obstacle_distance < this->sa_backward_limit_) {
		distance = -0.5f;
	} else {

		m = (y2 - y1)/(x2 - x1);
		b = y1 - m*x1;

		//m = (this->sa_distance_max_)/(this->sa_distance_max_ - this->sa_distance_zero_);
		//b = -m*this->sa_distance_zero_;

		distance = m*obstacle_distance + b;
		//distance = (obstacle_distance)*this->sa_distance_slope_ 
		//			- this->sa_distance_slope_*this->sa_distance_zero_;
	}

	if(distance > this->sa_distance_max_)
		distance = this->sa_distance_max_;

	ROS_INFO("Distance: %f\n", distance);

	return distance;

}

void SharedActions::init_command_timer(float timeout) {

	this->n_command_timer_ = this->nh_.createTimer(ros::Duration(this->n_command_timeout_), 
												   &SharedActions::on_reset_command_user,
												   this);
	ROS_WARN("Command timer initialize with %f timeout", this->n_command_timeout_);
}


void SharedActions::init_update_rate(float rate) {

	this->n_rate_ = new ros::Rate(rate);
}

void SharedActions::change_update_rate(float rate) {
	delete this->n_rate_;
	init_update_rate(rate);
}

void SharedActions::on_reset_command_user(const ros::TimerEvent& event) {

	ROS_WARN("User command validity expired");
	this->pr_attractors_.Reset();
	this->CancelGoal();
	this->n_command_timer_.stop();
}

bool SharedActions::on_requested_state_toggle(std_srvs::Trigger::Request& req,
											  std_srvs::Trigger::Response& res) {

	if(this->IsRunning() == true) {
		this->Stop();
		res.message = "Shared action has been stopped";
	} else {
		this->Start();
		res.message = "Shared action is running.";
		this->CancelGoal();
		this->MakeGoal();
		this->SendGoal();
	}
	res.success = true;

	return res.success;
}

bool SharedActions::on_requested_set_goal(std_srvs::Empty::Request& req,
										std_srvs::Empty::Response& res) {

	bool result = false;

	if(this->IsRunning() == true) {
		this->CancelGoal();
		this->MakeGoal();
		this->SendGoal();
		result = true;
	}
	
	return result;
}


void SharedActions::on_received_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Set true the availability of repellors data (used for first iteration)
	this->is_data_available_ = true;
	
	// Convert and store repellor data
	if(ProximitySectorConverter::FromMessage(data, this->pr_repellors_) == false) {
		ROS_ERROR("Cannot convert repellor proximity sector message");
	}
}

void SharedActions::on_received_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Convert and store attractor data
	if(ProximitySectorConverter::FromMessage(data, this->pr_attractors_) == false) {
		ROS_ERROR("Cannot convert attractors proximity sector message");
	}

	if(this->IsRunning() == false)
		return;

	// Update the timer for command timeout
	this->n_command_timer_.setPeriod(ros::Duration(this->n_command_timeout_));
	this->n_command_timer_.start();
}


geometry_msgs::PolygonStamped SharedActions::get_reset_area(float radius) {

	geometry_msgs::PolygonStamped area;
	geometry_msgs::Point32 point;
	unsigned int npoints = 100;

	for(auto i=0; i<npoints; i++) {
		point.x = radius*cos(2.0f*M_PI*i/(float)npoints);
		point.y = radius*sin(2.0f*M_PI*i/(float)npoints);
		point.z = 0.0f;

		area.polygon.points.push_back(point);
	}

	return area;
}

void SharedActions::on_received_odometry(const nav_msgs::Odometry& data) {
	this->m_odom_ = data;
}

bool SharedActions::is_within_reset_area(float radius) {

	float distance;
	bool result = false;

	geometry_msgs::Point pose;
	geometry_msgs::Point goal;

	pose = this->m_odom_.pose.pose.position;
	goal = this->m_goal_point_.point;

	distance = std::hypot((pose.x - goal.x), (pose.y - goal.y));

	if(distance <= radius)
		result = true;

	//ROS_INFO("Distance to next goal: %f", distance);

	return result;
}

    }
}

#endif
