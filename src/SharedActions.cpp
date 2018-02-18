#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_CPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_CPP

#include "cnbiros_shared_navigation/SharedActions.hpp"

namespace cnbiros {
    namespace navigation {

SharedActions::SharedActions(void) : private_nh_("~") {

    this->actioncln_ = nullptr;
    
	// Configure node
	this->configure();

	// Initialize services
	//this->srv_set_parameters_ = this->private_nh_.advertiseService("set_parameters", 
	//											  &SharedActions::on_set_parameters, this);

	this->srv_clear_costmap_ = this->nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// Initialize subscribers
    this->subrep_ = this->nh_.subscribe(this->reptopic_, 50, &SharedActions::on_receive_repellors, this);
    this->subatt_ = this->nh_.subscribe(this->atttopic_, 50, &SharedActions::on_receive_attractors, this);

	// Initialize move base client
    this->actioncln_ = new MoveBaseClient(this->actionsrv_, true);

	// Initialize user command timer
	this->init_command_timer(this->command_timeout_);

	// Initialize update rate
	this->init_update_rate(this->update_rate_);

	// Dynamic reconfiguration
	this->f = boost::bind(&SharedActions::reconfigure_callback, this, _1, _2);
	this->cfgserver.setCallback(f);

}

SharedActions::~SharedActions(void) {
    if(this->actioncln_ == nullptr)
	delete this->actioncln_;
}

void SharedActions::reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, 
										uint32_t level) {

	if(std::fabs(config.param_repellors_strength - this->repellors_strength_) > EPSILON) {
		this->repellors_strength_ = config.param_repellors_strength;
		ROS_WARN("Updated repellors strength to %f", this->repellors_strength_);
	}
	
	if(std::fabs(config.param_repellors_decay - this->repellors_decay_) > EPSILON) {
		this->repellors_decay_ = config.param_repellors_decay;
		ROS_WARN("Updated repellors decay to %f", this->repellors_decay_);
	}
	
	if(std::fabs(config.param_repellors_occupancy - this->repellors_occupancy_) > EPSILON) {
		this->repellors_occupancy_ = config.param_repellors_occupancy;
		ROS_WARN("Updated repellors occupancy to %f", this->repellors_occupancy_);
	}
	
	if(std::fabs(config.param_command_timeout - this->command_timeout_) > EPSILON) {
		this->command_timeout_ = config.param_command_timeout;
		ROS_WARN("Updated user's command timeout to %f", this->command_timeout_);
	}
	
	if(std::fabs(config.param_max_goal_distance - this->max_goal_distance_) > EPSILON) {
		this->max_goal_distance_ = config.param_max_goal_distance;
		ROS_WARN("Updated max user's goal distance to %f", this->max_goal_distance_);
	}
	
	if(std::fabs(config.param_slope_distance - this->slope_distance_) > EPSILON) {
		this->slope_distance_ = config.param_slope_distance;
		ROS_WARN("Updated max user's slope distance to %f", this->slope_distance_);
	}
	
	if(std::fabs(config.param_update_rate - this->update_rate_) > EPSILON) {
		this->update_rate_ = config.param_update_rate;
		this->change_update_rate(this->update_rate_);
		ROS_WARN("Updated rate to %f", this->update_rate_);
	}

}

bool SharedActions::configure(void) {

	this->reptopic_				= "/sectorgrid";
	this->atttopic_				= "/user";
	this->actionsrv_			= "move_base";
	this->repellors_strength_	= 0.2f;
	this->repellors_decay_		= 0.5f;
	this->repellors_occupancy_	= 9.5f;
	this->frame_id_				= "base_link";
	this->command_timeout_		= 8.0f;
	this->max_goal_distance_	= 2.0f;
	this->slope_distance_		= 5.0f;
	this->update_rate_			= 2.0f;


	// Getting parameters
	this->private_nh_.getParam("obstacles",				this->reptopic_);
	this->private_nh_.getParam("targets",				this->atttopic_);
	this->private_nh_.getParam("action_server",			this->actionsrv_);
	this->private_nh_.getParam("frame_id",				this->frame_id_);
	this->private_nh_.getParam("repellors_strength",	this->repellors_strength_);
	this->private_nh_.getParam("repellors_decay",		this->repellors_decay_);
	this->private_nh_.getParam("repellors_occupancy",	this->repellors_occupancy_);
	this->private_nh_.getParam("command_timeout",		this->command_timeout_);
	this->private_nh_.getParam("update_rate",			this->update_rate_);
	
	this->private_nh_.getParam("max_goal_distance",		this->max_goal_distance_);
	this->private_nh_.getParam("slope_distance",		this->slope_distance_);

	ROS_INFO("SharedActions frame_id:				%s", this->frame_id_.c_str());
	ROS_INFO("SharedActions repellors topic:		%s", this->reptopic_.c_str());
	ROS_INFO("SharedActions attractors topic:		%s", this->atttopic_.c_str());
	ROS_INFO("SharedActions action server name:		%s", this->actionsrv_.c_str());
	ROS_INFO("SharedActions repellors strength:		%f", this->repellors_strength_);
	ROS_INFO("SharedActions repellors decay:		%f", this->repellors_decay_);
	ROS_INFO("SharedActions repellors occupancy:	%f", this->repellors_occupancy_);
	ROS_INFO("SharedActions command timeout:		%f", this->command_timeout_);
	ROS_INFO("SharedActions max goal distance:		%f", this->max_goal_distance_);
	ROS_INFO("SharedActions slope distance:			%f", this->slope_distance_);
	ROS_INFO("SharedActions update rate:			%f", this->update_rate_);

	return true;
}

void SharedActions::init_command_timer(float timeout) {

	this->command_timer_ = this->nh_.createTimer(
						   ros::Duration(this->command_timeout_), 
						   &SharedActions::on_reset_command_user,
						   this);
	ROS_INFO("Command timer initialize with %f timeout", this->command_timeout_);
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

void SharedActions::WaitForServer(void) {

    while(!(this->actioncln_->waitForServer(ros::Duration(1.0f)))) {
		ROS_INFO_THROTTLE(5.0, "Waiting for %s action server to come up", this->actionsrv_.c_str());
	}
    ROS_INFO("%s action server connected", this->actionsrv_.c_str());
}

void SharedActions::Start(void) {

	float w = 0.0f;
	float r = 1.0f;

	this->MakeGoal(r, w);
	this->CancelServerGoal();
	this->SendGoal();
}

void SharedActions::MakeGoal(float radius, float angle) {

    // Make the goal for the given angle and radius
	this->goal_.target_pose.header.frame_id		= this->frame_id_;
    this->goal_.target_pose.pose.position.x		= radius*cos(angle);
    this->goal_.target_pose.pose.position.y		= radius*sin(angle);
    this->goal_.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(angle);
    this->goal_.target_pose.header.stamp		= ros::Time::now();
    	
	ROS_INFO("New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
}

void SharedActions::MakeGoal(void) {

	float angle;
	float map_angle;
	float usr_angle;
	float radius	= 1.0f;

	// Compute orientation for repellors
	map_angle  = this->goal_orientation_repellors(this->repellors_data_);
	ROS_INFO("Map angle: %f [deg]", map_angle*180.0f/M_PI);
	
	// Compute orientation for attractors
	usr_angle = this->goal_orientation_attractors(this->attractors_data_);
	ROS_INFO("Usr angle: %f [deg]", usr_angle*180.0f/M_PI);
	
	angle = map_angle + usr_angle;

	//////////// To be checked
	if (angle < -M_PI/2.0f) {
		angle = -M_PI/2.0f;
	} else if(angle > M_PI/2.0f) {
		angle = M_PI/2.0f;
	}
	//////////////////////////

	// Compute position for computed angle
	if(this->repellors_data_.IsEmpty() == false) {
		radius = this->goal_position_logistic(this->repellors_data_, angle);
	}
	
    // Make the goal given the computed angle and radius
	this->goal_.target_pose.header.frame_id		= this->frame_id_;
    this->goal_.target_pose.pose.position.x		= radius*cos(angle);
    this->goal_.target_pose.pose.position.y		= radius*sin(angle);
    this->goal_.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(angle);
    this->goal_.target_pose.header.stamp		= ros::Time::now();
    	
	ROS_INFO("New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
}


void SharedActions::SendGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
    	this->actioncln_->sendGoal(this->goal_);
	}
}

void SharedActions::CancelServerGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
		this->actioncln_->cancelGoal();
	}
}


void SharedActions::on_receive_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Convert and store repellor data
	if(ProximitySectorConverter::FromMessage(data, this->repellors_data_) == false) {
		ROS_ERROR("Cannot convert repellor proximity sector message");
	}
}

void SharedActions::on_receive_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data) {

	// Convert and store attractor data
	if(ProximitySectorConverter::FromMessage(data, this->attractors_data_) == false) {
		ROS_ERROR("Cannot convert attractors proximity sector message");
	}

	// Update the timer for command timeout
	this->command_timer_.setPeriod(ros::Duration(this->command_timeout_));
	this->command_timer_.start();

	// Force new goal immediately
	this->MakeGoal();
	this->CancelServerGoal();
	this->SendGoal();
}

float SharedActions::goal_orientation_repellors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
    float sector_step;
    float cradius, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

    sector_step	= sectors.GetStep();

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		cradius = sectors.GetRadius(it);
		cangle  = sectors.GetAngle(it);

		clambda = this->repellors_strength_*exp(-(cradius/this->repellors_decay_));
		csigma  = std::atan(std::tan(sector_step/2.0f) + 
			           this->repellors_occupancy_/(this->repellors_occupancy_ + cradius));
		
		w += clambda*(hangle-cangle)*exp(-(std::pow(hangle - cangle, 2))/(2.0f*pow(csigma, 2)));
    }

    return w;
}

float SharedActions::goal_orientation_attractors(ProximitySector& sectors) {

    float w;
	ProximitySectorConstIt it;

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		w  = -sectors.GetAngle(it);
    }
    return w;
}

float SharedActions::goal_position_exponential(ProximitySector& sectors, float wescape) {

	float cvalue;
	float tposition;
    float a;

    float MINDIS = 0.0f;
    float MAXDIS = 6.0f;
    float MAXPOS = 1.0f;
    
    a = MAXPOS/std::pow((MAXDIS-MINDIS), 2);

	cvalue = sectors.At(wescape);

    if(cvalue <= MINDIS) {
		tposition = 0.0f;
    } else if(std::isinf(cvalue) || cvalue > MAXDIS) {
		tposition = MAXPOS;
    } else {
		tposition = a*std::pow((cvalue-MINDIS), 2);
    }

	return tposition;
}

float SharedActions::goal_position_logistic(ProximitySector& sectors, float wescape) {

	float cvalue;
	float tposition;

    float MaxDg  = this->max_goal_distance_;
	float MinDg  = 0.01f;
	float Slope  = this->slope_distance_;
	float MidPoint = 2.0f;
	float ShiftX = (1.0f/std::exp(Slope))*((2.0f/MinDg) - 1.0f);

	cvalue = sectors.At(wescape);
	
	if(std::isinf(cvalue)) {
		tposition = MaxDg;
	} else {
		tposition = MaxDg/(1+ShiftX*std::exp(-Slope*(cvalue - MidPoint)));
	}

	return tposition;

}
/*
float SharedActions::compute_position_linear(const cnbiros_shared_navigation::SectorGrid& data, float wescape) {

	unsigned int idsector;
	float cvalue;
	float tposition;

    float m, q;
    float MINDIS = 0.8f;
    float MAXDIS = 10.0f;
    float MINPOS = 0.5f; 
    float MAXPOS = 1.0f;
    
    
	m = (MAXPOS - MINPOS)/(MAXDIS - MINDIS);
    q = (MAXDIS*MINPOS - MAXPOS*MINDIS)/(MAXDIS - MINDIS);
	
	idsector = angle2sector(wescape, data.min_angle, data.max_angle, data.step, data.nsectors);
	cvalue = data.values.at(idsector);

    if(cvalue <= MINDIS) {
		tposition = 0.0f;
    } else if(std::isinf(cvalue)) {
		tposition = MAXPOS;
    } else {
		tposition = cvalue*m + q;
    }

	return tposition;

}
*/
/*
unsigned int SharedActions::angle2sector(float angle, float min_angle, float max_angle, float step_angle, unsigned int nsectors) {
	
	unsigned int idsector;

	if(angle == min_angle) {
		idsector = 0;
	} else if(angle == max_angle) {
		idsector = nsectors-1;
	} else {
		idsector = std::floor((angle - min_angle)/step_angle);
	}

	return idsector;
}
*/

/*
float SharedActions::compute_position(const cnbiros_shared_navigation::SectorGrid& data) {
    
    float tposition = 1.0f;
    float cradius, cangle;
    float width;
    float min_obstacle_distance;
    float sector_step, sector_min_angle;
    unsigned int i;

    std::vector<float> inrange;
    std::vector<float>::iterator itmin;
    width = 0.8f;

    float MINDIS = 0.8f;
    float MAXDIS = 10.0f;
    float MINPOS = 0.5f; 
    float MAXPOS = 1.0f;
    
    float m, q;
    float obstacle_distance, cprojection;
    obstacle_distance = std::numeric_limits<float>::infinity();
    
    sector_step	     = data.step;
    sector_min_angle = data.min_angle;

    m = (MAXPOS - MINPOS)/(MAXDIS - MINDIS);
    q = (MAXDIS*MINPOS - MAXPOS*MINDIS)/(MAXDIS - MINDIS);

    i = 0;
    for(auto it=data.values.begin(); it!=data.values.end(); ++it) {
		i++;
		if(std::isinf((*it)) == true)
		    continue;

		cradius = (*it);
		cangle  = sector_min_angle + sector_step*((i-1) + 0.5f);

		cprojection = std::abs(std::cos(cangle));
		
		if(cprojection >= width/2.0f)
		    inrange.push_back(cradius);
    }

    if(inrange.empty() == false) {
		itmin = std::min_element(std::begin(inrange), std::end(inrange));
		obstacle_distance = (*itmin);
    }

    if(obstacle_distance <= MINDIS) {
		tposition = 0.0f;
    } else if(std::isinf(obstacle_distance)) {
		tposition = MAXPOS;
    } else {
		tposition = obstacle_distance*m + q;
    }


    return tposition;
}
*/
/*
bool SharedActions::on_set_parameters(cnbiros_shared_navigation::DynamicGoalsParameters::Request& req,
									 cnbiros_shared_navigation::DynamicGoalsParameters::Response& res) {

	res.result = true;

	if( req.name.compare("obstacle_strength") == 0 ) {
		if(req.value <= 0.0f) {
			ROS_ERROR("Obstacle strength must be > 0.0f");
			res.result = false;
		} else {
			this->obstacle_strength_ = req.value;
			ROS_INFO("Updated obstacle strength to: %f", this->obstacle_strength_);
		}
	} else if( req.name.compare("obstacle_decay") == 0 ) {
		if(req.value <= 0.0f) {
			ROS_ERROR("Obstacle decay must be > 0.0f");
			res.result = false;
		} else {
			this->obstacle_decay_ = req.value;
			ROS_INFO("Updated obstacle decay to: %f", this->obstacle_decay_);
		}
	} else if( req.name.compare("obstacle_occupancy") == 0 ) {
		if(req.value <= 0.0f) {
			ROS_ERROR("Obstacle occupancy must be > 0.0f");
			res.result = false;
		} else {
			this->obstacle_occupancy_ = req.value;
			ROS_INFO("Updated obstacle occupancy to: %f", this->obstacle_occupancy_);
		}
	} else if( req.name.compare("command_timeout") == 0 ) {
		this->command_timeout_ = req.value;
		ROS_INFO("Updated user command timeout to: %f", this->command_timeout_);
	} else if( req.name.compare("max_goal_distance") == 0 ) {
		this->max_goal_distance_ = req.value;
		ROS_INFO("Updated max goal distance to: %f", this->max_goal_distance_);
	} else if( req.name.compare("slope_distance") == 0 ) {
		this->slope_distance_ = req.value;
		ROS_INFO("Updated slope distance distance to: %f", this->slope_distance_);
	} else {
		ROS_ERROR("'%s' is not a parameter of DynamicGoal node", req.name.c_str());
		res.result = false;
	}

	return res.result;
}
*/

void SharedActions::Run(void) {
   
	actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::LOST);

	std_srvs::Empty srvmsg_clearcostmap;

    while(this->nh_.ok()) {

		if(this->actioncln_->isServerConnected()) {
			
			state = this->actioncln_->getState();
		
			// Check the current status
			// If it is ABORTED or LOST, then clear costmap
			if((state == actionlib::SimpleClientGoalState::ABORTED) ||
			   (state == actionlib::SimpleClientGoalState::LOST) ) {
				ROS_WARN("Goal aborted/lost. Clearing costmap...");

				if(this->srv_clear_costmap_.call(srvmsg_clearcostmap)) {
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
			this->CancelServerGoal();
			this->SendGoal();



			//if( (state != actionlib::SimpleClientGoalState::LOST) &&
			//	(state != actionlib::SimpleClientGoalState::PENDING) &&
			//	(state != actionlib::SimpleClientGoalState::ACTIVE) ) {
			//	ROS_INFO_THROTTLE(2, "%s", state.toString().c_str());
			//}

			//if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			//	// service clear map
			//	ROS_INFO("Goal reached. Forcing new goal");

			//	this->MakeGoal();
			//	this->CancelServerGoal();
			//	this->SendGoal();
			//} else if (state == actionlib::SimpleClientGoalState::ABORTED) {
			//	ROS_INFO("Goal aborted. Clearing costmap and forcing new goal");

			//	if(this->srv_clear_costmap_.call(srvmsg_clearcostmap)) {
			//		ROS_INFO("Costmap cleared");
			//	} else {
			//		ROS_ERROR("Failed to request costmap clearing");
			//	}

			//	this->MakeGoal();
			//	this->CancelServerGoal();
			//	this->SendGoal();
			//}

		}
		ros::spinOnce();
		this->rate_->sleep();
    }

}

    }
}

#endif
