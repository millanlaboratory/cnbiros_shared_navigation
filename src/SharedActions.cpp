#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_CPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_CPP

#include "cnbiros_shared_navigation/SharedActions.hpp"

namespace cnbiros {
    namespace navigation {

SharedActions::SharedActions(void) : private_nh_("~") {

    this->actioncln_ = nullptr;
    
	// Initialize user command timer
	this->init_command_timer(1.0f);

	// Initialize update rate
	this->init_update_rate(1.0f);
	
	// Dynamic reconfiguration
	this->f_ = boost::bind(&SharedActions::reconfigure_callback, this, _1, _2);
	this->cfgserver_.setCallback(this->f_);
	
	// Configure node
	this->configure();

	// Initialize subscribers
    this->subrep_ = this->nh_.subscribe(this->reptopic_, 50, &SharedActions::on_receive_repellors, this);
    this->subatt_ = this->nh_.subscribe(this->atttopic_, 50, &SharedActions::on_receive_attractors, this);

	// Initialize move base client
    this->actioncln_ = new MoveBaseClient(this->actionsrv_, true);

	// Initialize clearcostmap service
	this->srv_clearcostmap_ = this->nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// Initialize start/stop service server
	this->srv_state_toggle_ = this->private_nh_.advertiseService("toggle_state", &SharedActions::on_request_state_toggle, this);
	

	// If is running (autostart)
	if(this->autostart_)
		this->Start(); 
}

SharedActions::~SharedActions(void) {
    if(this->actioncln_ == nullptr)
	delete this->actioncln_;
}


bool SharedActions::configure(void) {

	// Getting parameters
	this->private_nh_.param<std::string>("repellors_source", this->reptopic_, "/repellors");
	this->private_nh_.param<std::string>("attractors_source", this->atttopic_, "/attractors");
	this->private_nh_.param<std::string>("action_server", this->actionsrv_, "move_base");
	this->private_nh_.param<std::string>("frame_id", this->frame_id_, "base_link");
	this->private_nh_.param<float>("robot_radius", this->robot_radius_, 0.52f);
	this->private_nh_.param<float>("repellors_strength", this->repellors_strength_, 0.2f);
	this->private_nh_.param<float>("repellors_decay", this->repellors_decay_, 0.5f);
	this->private_nh_.param<float>("goal_max_distance", this->goal_max_distance_, 2.0f);
	this->private_nh_.param<float>("goal_half_position", this->goal_half_position_, 1.5f);
	this->private_nh_.param<float>("command_timeout", this->command_timeout_, 8.0f);
	this->private_nh_.param<float>("update_rate", this->update_rate_, 2.0f);
	this->private_nh_.param<bool>("autostart", this->autostart_, false);

	// Dump parameters
	ROS_INFO("SharedActions frame_id:				%s", this->frame_id_.c_str());
	ROS_INFO("SharedActions repellors topic:		%s", this->reptopic_.c_str());
	ROS_INFO("SharedActions attractors topic:		%s", this->atttopic_.c_str());
	ROS_INFO("SharedActions action server name:		%s", this->actionsrv_.c_str());
	ROS_INFO("SharedActions robot circumscribed radius:	%f", this->robot_radius_);
	ROS_INFO("SharedActions repellors strength:		%f", this->repellors_strength_);
	ROS_INFO("SharedActions repellors decay:		%f", this->repellors_decay_);
	ROS_INFO("SharedActions goal max distance:		%f", this->goal_max_distance_);
	ROS_INFO("SharedActions goal half position:		%f", this->goal_half_position_);
	ROS_INFO("SharedActions command timeout:		%f", this->command_timeout_);
	ROS_INFO("SharedActions update rate:			%f", this->update_rate_);

	// Initialize to false availability of repellors_data
	this->is_data_available_ = false;

	// Initialize is running to false
	this->is_running_ = false;

	return true;
}

bool SharedActions::IsRunning(void) {
	return this->is_running_;
}

void SharedActions::WaitForServer(void) {

    while(!(this->actioncln_->waitForServer(ros::Duration(1.0f)))) {
		ROS_INFO_THROTTLE(5.0, "Waiting for %s action server to come up", this->actionsrv_.c_str());
	}
    ROS_INFO("%s action server connected", this->actionsrv_.c_str());
}

void SharedActions::Start(void) {
	if(this->IsRunning() == false) {
		this->is_running_ = true;
		ROS_WARN("SharedActions started");
	}
}

void SharedActions::Stop(void) {
	if(this->IsRunning() == true) {
		this->is_running_ = false;
		ROS_WARN("SharedActions stopped");
	}
}

void SharedActions::MakeGoal(void) {

	float angle;
	float repangle;
	float attangle;
	float radius = 0.0f;

	
	// Compute orientation for repellors
	repangle  = this->goal_orientation_repellors(this->repellors_data_);
	ROS_DEBUG_NAMED("sharedactions", "Map angle (heading): %f [deg]", repangle*180.0f/M_PI);
	
	// Compute orientation for attractors
	attangle = this->goal_orientation_attractors(this->attractors_data_);
	ROS_DEBUG_NAMED("sharedactions", "Usr angle (heading): %f [deg]", attangle*180.0f/M_PI);
	
	angle = repangle + attangle;

	// Limit the orientation to the front
	angle = this->goal_orientation_limits(angle, -M_PI/2.0f, M_PI/2.0f);	
	
	// Compute position for computed angle (if repellors_data_ is available)
	if(this->is_data_available_ == true) {
		radius = this->goal_position_logistic(this->repellors_data_, angle);
	}

	// Rotate angle (standard coordinate frame conventions)	
	angle = M_PI/2.0f + angle;

    // Make the goal given the computed angle and radius
	this->goal_.target_pose.header.frame_id		= this->frame_id_;
    this->goal_.target_pose.pose.position.y		= -radius*cos(angle);
    this->goal_.target_pose.pose.position.x		= radius*sin(angle);
    this->goal_.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(angle-M_PI/2.0f);
    this->goal_.target_pose.header.stamp		= ros::Time::now();
    	
	//ROS_DEBUG_NAMED("sharedactions", "New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
	ROS_INFO("New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
}


void SharedActions::SendGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
    	this->actioncln_->sendGoal(this->goal_);
	}
}

void SharedActions::CancelGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
		this->actioncln_->cancelGoal();
	}
}

void SharedActions::Run(void) {
   
	actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::LOST);
	std_srvs::Empty emptymsg;

    while(this->nh_.ok()) {
		
		ros::spinOnce();
		this->rate_->sleep();

		if(this->is_running_ != true) {
			ROS_WARN_THROTTLE(5, "SharedAction waiting to be started...");
			continue;
		}

		if(this->actioncln_->isServerConnected()) {
			
			state = this->actioncln_->getState();
		
			// Check the current status
			// If it is ABORTED or LOST, then clear costmap
			if((state == actionlib::SimpleClientGoalState::ABORTED) ||
			   (state == actionlib::SimpleClientGoalState::LOST) ) {
				ROS_WARN("Goal aborted/lost. Clearing costmap...");

				if(this->srv_clearcostmap_.call(emptymsg)) {
					ROS_WARN("Costmap cleared");
				} else {
					ROS_ERROR("Failed to request costmap clearing");
				}
			}

			// Force a new goal and cancel the previous ones (semi-autonomous
			// behaviour).
			// The new goal is based on the last costmap received and on the
			// last user command received (if exists)
			this->MakeGoal();
			this->CancelGoal();
			this->SendGoal();
		}
    }

}


float SharedActions::goal_orientation_repellors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
    float sector_step;
    float cradius, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

	float robotsize = 2.0f*(this->robot_radius_);

    sector_step	= sectors.GetStep();

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		cradius = sectors.GetRadius(it);
		cangle  = sectors.GetAngle(it);

		clambda = this->repellors_strength_*exp(-(cradius/this->repellors_decay_));
		csigma  = std::atan(std::tan(sector_step/2.0f) + 
			           robotsize/(robotsize + cradius));
		
		w += clambda*(hangle-cangle)*exp(-(std::pow(hangle - cangle, 2))/(2.0f*pow(csigma, 2)));
    }

    return w;
}

float SharedActions::goal_orientation_attractors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
    float w = 0.0f;

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		w  += sectors.GetAngle(it);
    }
    return w;
}

float SharedActions::goal_orientation_limits(float angle, float minangle, float maxangle) {

	float langle;

	langle = angle;
	if (angle < minangle) {
		langle = minangle;
	} else if(angle > maxangle) {
		langle = maxangle;
	}

	return langle;
}

float SharedActions::goal_position_logistic(ProximitySector& sectors, float wescape) {

	float cvalue = std::numeric_limits<float>::infinity();
	float goaldistance;

    float MaxPosition = this->goal_max_distance_;
	float MinPosition = SHAREDACTIONS_LOGISTIC_MINDISTANCE;
	float MinDistance = this->robot_radius_;
	float HalfPosition = this->goal_half_position_;
	float Slope;

	Slope = -std::log((MaxPosition/MinPosition) - 1.0f)*(1.0f/(MinDistance - HalfPosition));

	for(auto it=sectors.Begin(); it!=sectors.End(); ++it)
		cvalue = std::min(cvalue, (*it));

	//try {
	//	cvalue = sectors.At(wescape);
	//} catch (std::runtime_error e) {
	//	ROS_ERROR("Error: %s", e.what());
	//}
	
	if(std::isinf(cvalue)) {
		goaldistance = MaxPosition;
	} else {
		goaldistance = MaxPosition/(1.0f + std::exp(-Slope*(cvalue-HalfPosition)));
	}

	return goaldistance;

}

void SharedActions::reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, 
										uint32_t level) {

	if(std::fabs(config.param_repellors_strength - this->repellors_strength_) > 0.00001f) {
		this->repellors_strength_ = config.param_repellors_strength;
		ROS_WARN("Updated repellors strength to %f", this->repellors_strength_);
	}
	
	if(std::fabs(config.param_repellors_decay - this->repellors_decay_) > 0.00001f) {
		this->repellors_decay_ = config.param_repellors_decay;
		ROS_WARN("Updated repellors decay to %f", this->repellors_decay_);
	}
	
	if(std::fabs(config.param_command_timeout - this->command_timeout_) > 0.00001f) {
		this->command_timeout_ = config.param_command_timeout;
		ROS_WARN("Updated user's command timeout to %f", this->command_timeout_);
	}
	
	if(std::fabs(config.param_goal_max_distance - this->goal_max_distance_) > 0.00001f) {
		this->goal_max_distance_ = config.param_goal_max_distance;
		ROS_WARN("Updated goal max distance to %f", this->goal_max_distance_);
	}
	
	if(std::fabs(config.param_goal_half_position - this->goal_half_position_) > 0.00001f) {
		this->goal_half_position_ = config.param_goal_half_position;
		ROS_WARN("Updated goal half position to %f", this->goal_half_position_);
	}
	
	if(std::fabs(config.param_update_rate - this->update_rate_) > 0.00001f) {
		this->update_rate_ = config.param_update_rate;
		this->change_update_rate(this->update_rate_);
		ROS_WARN("Updated rate to %f", this->update_rate_);
	}

}

void SharedActions::init_command_timer(float timeout) {

	this->command_timer_ = this->nh_.createTimer(
						   ros::Duration(this->command_timeout_), 
						   &SharedActions::on_reset_command_user,
						   this);
	ROS_WARN("Command timer initialize with %f timeout", this->command_timeout_);
}


void SharedActions::init_update_rate(float rate) {

	this->rate_ = new ros::Rate(rate);
}

void SharedActions::change_update_rate(float rate) {
	delete this->rate_;
	init_update_rate(rate);
}

void SharedActions::on_reset_command_user(const ros::TimerEvent& event) {

	ROS_WARN("User command validity expired");
	this->attractors_data_.Reset();

	this->command_timer_.stop();
}

bool SharedActions::on_request_state_toggle(std_srvs::Trigger::Request& req,
											std_srvs::Trigger::Response& res) {

	if(this->IsRunning() == true) {
		this->Stop();
		res.message = "Shared action has been stopped";
	} else {
		this->Start();
		res.message = "Shared action is running.";
	}
	res.success = true;

	return res.success;
}


void SharedActions::on_receive_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Set true the availability of repellors data (used for first iteration)
	this->is_data_available_ = true;
	
	// Convert and store repellor data
	if(ProximitySectorConverter::FromMessage(data, this->repellors_data_) == false) {
		ROS_ERROR("Cannot convert repellor proximity sector message");
	}

	//this->repellors_data_.Dump();
}

void SharedActions::on_receive_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Convert and store attractor data
	if(ProximitySectorConverter::FromMessage(data, this->attractors_data_) == false) {
		ROS_ERROR("Cannot convert attractors proximity sector message");
	}

	if(this->is_running_ == false)
		return;

	// Update the timer for command timeout
	this->command_timer_.setPeriod(ros::Duration(this->command_timeout_));
	this->command_timer_.start();

	//// Force new goal immediately
	//this->MakeGoal();
	//this->CancelGoal();
	//this->SendGoal();
}
    }
}

#endif
