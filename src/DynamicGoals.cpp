#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_CPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_CPP

#include "cnbiros_wheelchair_navigation/DynamicGoals.hpp"

namespace cnbiros {
    namespace navigation {

DynamicGoals::DynamicGoals(void) : private_nh_("~") {

    this->actioncln_ = nullptr;
    
	// Configure node
	this->configure();

	// Initialize services
	this->srv_set_parameters_ = this->private_nh_.advertiseService("set_parameters", 
												  &DynamicGoals::on_set_parameters, this);

	this->srv_clear_costmap_ = this->nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// Initialize subscribers
    this->subobstacles_ = this->nh_.subscribe(this->obstacle_topic_, 50, &DynamicGoals::callback, this);
    this->subtargets_   = this->nh_.subscribe(this->target_topic_, 50, &DynamicGoals::user_callback, this);

	// Initialize move base client
    this->actioncln_ = new MoveBaseClient(this->actionsrv_, true);

	// Initialize user command timer
	this->init_command_timer(this->command_timeout_);
}

DynamicGoals::~DynamicGoals(void) {
    if(this->actioncln_ == nullptr)
	delete this->actioncln_;
}

bool DynamicGoals::configure(void) {

	this->obstacle_strength_	= 1.0f;
	this->obstacle_decay_		= 1.0f;
	this->obstacle_occupancy_	= 50.0f;
	this->obstacle_topic_		= "/sectorgrid";
	this->target_topic_			= "/user";
	this->actionsrv_			= "move_base";
	this->frame_id_				= "base_link";
	this->command_timeout_		= 5.0f;
	this->max_goal_distance_	= 1.0f;
	this->slope_distance_		= 3.0f;

	this->is_map_available_		= false;

	// Getting parameters
	this->private_nh_.getParam("obstacles",				this->obstacle_topic_);
	this->private_nh_.getParam("targets",				this->target_topic_);
	this->private_nh_.getParam("action_server",			this->actionsrv_);
	this->private_nh_.getParam("frame_id",				this->frame_id_);
	this->private_nh_.getParam("obstacle_strength",		this->obstacle_strength_);
	this->private_nh_.getParam("obstacle_decay",		this->obstacle_decay_);
	this->private_nh_.getParam("obstacle_occupancy",	this->obstacle_occupancy_);
	this->private_nh_.getParam("command_timeout",		this->command_timeout_);
	
	this->private_nh_.getParam("max_goal_distance",		this->max_goal_distance_);
	this->private_nh_.getParam("slope_distance",		this->slope_distance_);

	ROS_INFO("DynamicGoals frame_id:					%s", this->frame_id_.c_str());
	ROS_INFO("DynamicGoals obstacles topic:				%s", this->obstacle_topic_.c_str());
	ROS_INFO("DynamicGoals target topic:				%s", this->target_topic_.c_str());
	ROS_INFO("DynamicGoals action server name:			%s", this->actionsrv_.c_str());
	ROS_INFO("DynamicGoals obstacles strength:			%f", this->obstacle_strength_);
	ROS_INFO("DynamicGoals obstacles decay:				%f", this->obstacle_decay_);
	ROS_INFO("DynamicGoals obstacles occupancy:			%f", this->obstacle_occupancy_);
	ROS_INFO("DynamicGoals command timeout:				%f", this->command_timeout_);
	ROS_INFO("DynamicGoals max goal distance:			%f", this->max_goal_distance_);
	ROS_INFO("DynamicGoals slope distance:				%f", this->slope_distance_);

	return true;
}

void DynamicGoals::init_command_timer(float timeout) {

	this->command_timer_ = this->nh_.createTimer(
						   ros::Duration(this->command_timeout_), 
						   &DynamicGoals::on_reset_command_user,
						   this);
	ROS_INFO("Command timer initialize with %f timeout", this->command_timeout_);
}

void DynamicGoals::on_reset_command_user(const ros::TimerEvent& event) {

	ROS_WARN("User command validity expired");
	this->user_data_.values.clear();

	this->command_timer_.stop();
}

void DynamicGoals::WaitForServer(void) {

    while(!(this->actioncln_->waitForServer(ros::Duration(1.0f)))) {
		ROS_INFO_THROTTLE(5.0, "Waiting for %s action server to come up", this->actionsrv_.c_str());
	}
    ROS_INFO("%s action server connected", this->actionsrv_.c_str());
}

void DynamicGoals::Start(void) {

	float w = 0.0f;
	float r = 1.0f;

	this->MakeGoal(r, w);
	this->CancelServerGoal();
	this->SendGoal();
}

void DynamicGoals::MakeGoal(float radius, float angle) {

    // Make the goal for the given angle and radius
	this->goal_.target_pose.header.frame_id		= this->frame_id_;
    this->goal_.target_pose.pose.position.x		= radius*cos(angle);
    this->goal_.target_pose.pose.position.y		= radius*sin(angle);
    this->goal_.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(angle);
    this->goal_.target_pose.header.stamp		= ros::Time::now();
    	
	ROS_INFO("New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
}

void DynamicGoals::MakeGoal(void) {

	float angle;
	float map_angle;
	float usr_angle;
	float radius	= 1.0f;

	// Compute orientation for repellors
	map_angle  = this->compute_orientation(this->sector_data_);
	ROS_INFO("Map angle: %f [deg]", map_angle*180.0f/M_PI);
	
	// Compute orientation for attractors
	usr_angle = this->compute_orientation_user(this->user_data_);
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
	if(this->sector_data_.values.empty() == false) {
		//radius = this->compute_position_exponential(this->sector_data_, angle);
		radius = this->compute_position_logistic(this->sector_data_, angle);
	}
	
    // Make the goal given the computed angle and radius
	this->goal_.target_pose.header.frame_id		= this->frame_id_;
    this->goal_.target_pose.pose.position.x		= radius*cos(angle);
    this->goal_.target_pose.pose.position.y		= radius*sin(angle);
    this->goal_.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(angle);
    this->goal_.target_pose.header.stamp		= ros::Time::now();
    	
	ROS_INFO("New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
}


void DynamicGoals::SendGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
    	this->actioncln_->sendGoal(this->goal_);
	}
}

void DynamicGoals::CancelServerGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
		this->actioncln_->cancelGoal();
	}
}


void DynamicGoals::callback(const cnbiros_wheelchair_navigation::SectorGrid& data) {

    this->sector_data_ = data;
	this->is_map_available_ = true;

	this->MakeGoal();
	this->CancelServerGoal();
	this->SendGoal();
}

void DynamicGoals::user_callback(const cnbiros_wheelchair_navigation::SectorGrid& data) {

    this->user_data_ = data;
	this->command_timer_.setPeriod(ros::Duration(this->command_timeout_));
	this->command_timer_.start();

	this->MakeGoal();
	this->CancelServerGoal();
	this->SendGoal();
}

float DynamicGoals::compute_orientation(const cnbiros_wheelchair_navigation::SectorGrid& data) {

    unsigned int i;
    unsigned int nsectors;
    float sector_step, sector_min_angle, sector_max_angle;
    float cradius, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

    sector_step	     = data.step;
    sector_min_angle = data.min_angle;
    sector_max_angle = data.max_angle;
    nsectors	     = data.nsectors;

    i = 0;
    for(auto it=data.values.begin(); it!=data.values.end(); ++it) {
		i++;

		if(std::isinf((*it)) == true)
		    continue;

		cradius = (*it);
		cangle  = sector_min_angle + sector_step*((i-1) + 0.5f);
		
		//ROS_INFO("Obstacle at: %f degree", cangle*180.0f/M_PI);	
		clambda = this->obstacle_strength_*exp(-(cradius/this->obstacle_decay_));
		csigma  = std::atan(std::tan(sector_step/2.0f) + 
			           this->obstacle_occupancy_/(this->obstacle_occupancy_ + cradius));
		
		w += clambda*(hangle-cangle)*exp(-(std::pow(hangle - cangle, 2))/(2.0f*pow(csigma, 2)));

    }


    return w;

}

float DynamicGoals::compute_orientation_user(const cnbiros_wheelchair_navigation::SectorGrid& data) {

    unsigned int i;
    unsigned int nsectors;
    float sector_step, sector_min_angle, sector_max_angle;
    float cradius, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

    sector_step	     = data.step;
    sector_min_angle = data.min_angle;
    sector_max_angle = data.max_angle;
    nsectors	     = data.nsectors;


    i = 0;
    for(auto it=data.values.begin(); it!=data.values.end(); ++it) {
		i++;

		if(std::isinf((*it)) == true)
		    continue;

		cradius = (*it);
		cangle  = sector_min_angle + sector_step*((i-1) + 0.5f);
		
		ROS_INFO("User command at: %f degree", cangle*180.0f/M_PI);	
		
		w = -cangle;
    }

    return w;
}

float DynamicGoals::compute_position_exponential(const cnbiros_wheelchair_navigation::SectorGrid& data, float wescape) {

	unsigned int idsector;
	float cvalue;
	float tposition;

    float a;
    float MINDIS = 0.0f;
    float MAXDIS = 6.0f;
    float MAXPOS = 1.0f;
    
    a = MAXPOS/std::pow((MAXDIS-MINDIS), 2);

	
	idsector = angle2sector(wescape, data.min_angle, data.max_angle, data.step, data.nsectors);
	cvalue = data.values.at(idsector);

    if(cvalue <= MINDIS) {
		tposition = 0.0f;
    } else if(std::isinf(cvalue) || cvalue > MAXDIS) {
		tposition = MAXPOS;
    } else {
		tposition = a*std::pow((cvalue-MINDIS), 2);
    }

	return tposition;

}

float DynamicGoals::compute_position_logistic(const cnbiros_wheelchair_navigation::SectorGrid& data, float wescape) {

	unsigned int idsector;
	float cvalue;
	float tposition;

    float MaxDg  = this->max_goal_distance_;
	float MinDg  = 0.01f;
	float Slope  = this->slope_distance_;
	float MidPoint = 2.0f;
	float ShiftX = (1.0f/std::exp(Slope))*((2.0f/MinDg) - 1.0f);

	idsector = angle2sector(wescape, data.min_angle, data.max_angle, data.step, data.nsectors);
	cvalue = data.values.at(idsector);

	if(std::isinf(cvalue)) {
		tposition = MaxDg;
	} else {
		tposition = MaxDg/(1+ShiftX*std::exp(-Slope*(cvalue - MidPoint)));
	}

	return tposition;

}
/*
float DynamicGoals::compute_position_linear(const cnbiros_wheelchair_navigation::SectorGrid& data, float wescape) {

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
unsigned int DynamicGoals::angle2sector(float angle, float min_angle, float max_angle, float step_angle, unsigned int nsectors) {
	
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

/*
float DynamicGoals::compute_position(const cnbiros_wheelchair_navigation::SectorGrid& data) {
    
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
bool DynamicGoals::on_set_parameters(cnbiros_wheelchair_navigation::DynamicGoalsParameters::Request& req,
									 cnbiros_wheelchair_navigation::DynamicGoalsParameters::Response& res) {

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

void DynamicGoals::Run(void) {
   
	actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::LOST);

	std_srvs::Empty srvmsg_clearcostmap;

    while(this->nh_.ok()) {

		if(this->actioncln_->isServerConnected()) {
			
			state = this->actioncln_->getState();
			
			if( (state != actionlib::SimpleClientGoalState::LOST) &&
				(state != actionlib::SimpleClientGoalState::PENDING) &&
				(state != actionlib::SimpleClientGoalState::ACTIVE) ) {
				ROS_INFO_THROTTLE(2, "%s", state.toString().c_str());
			}

			if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				// service clear map
				ROS_INFO("Goal reached. Forcing new goal");

				this->MakeGoal();
				this->CancelServerGoal();
				this->SendGoal();
			} else if (state == actionlib::SimpleClientGoalState::ABORTED) {
				ROS_INFO("Goal aborted. Clearing costmap and forcing new goal");

				if(this->srv_clear_costmap_.call(srvmsg_clearcostmap)) {
					ROS_INFO("Costmap cleared");
				} else {
					ROS_ERROR("Failed to request costmap clearing");
				}

				this->MakeGoal();
				this->CancelServerGoal();
				this->SendGoal();
			}

		}
		ros::spinOnce();
    }

}

    }
}

#endif
