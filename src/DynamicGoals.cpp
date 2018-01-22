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
	//this->srv_obstacle_strength_ = this->private_nh_.advertiseService("set_obstacle_strength", 
	//											  &DynamicGoals::on_set_obstacle_strength, this);
	//this->srv_obstacle_decay_	 = this->private_nh_.advertiseService("set_obstacle_decay", 
	//											  &DynamicGoals::on_set_obstacle_decay, this);
	this->srv_set_parameters_ = this->private_nh_.advertiseService("set_parameters", 
												  &DynamicGoals::on_set_parameters, this);

	this->srv_clear_costmap_ = this->nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// Initialize subscribers
    this->subobstacles_ = this->nh_.subscribe(this->obstacle_topic_, 50, &DynamicGoals::callback, this);
    this->subtargets_   = this->nh_.subscribe(this->target_topic_, 50, &DynamicGoals::callback, this);

	// Initialize move base client
    this->actioncln_ = new MoveBaseClient(this->actionsrv_, true);

}

DynamicGoals::~DynamicGoals(void) {
    if(this->actioncln_ == nullptr)
	delete this->actioncln_;
}

bool DynamicGoals::configure(void) {

	this->obstacle_strength_	= 4.0f;
	this->obstacle_decay_		= 2.0f;
	this->obstacle_occupancy_	= 1.0f;
	this->obstacle_topic_		= "/sectorgrid";
	this->target_topic_			= "/tmpname";
	this->actionsrv_			= "move_base";
	this->frame_id_				= "base_link";

	// Getting parameters
	this->private_nh_.getParam("obstacles",				this->obstacle_topic_);
	this->private_nh_.getParam("targets",				this->target_topic_);
	this->private_nh_.getParam("action_server",			this->actionsrv_);
	this->private_nh_.getParam("frame_id",				this->frame_id_);
	this->private_nh_.getParam("obstacle_strength",		this->obstacle_strength_);
	this->private_nh_.getParam("obstacle_decay",		this->obstacle_decay_);
	this->private_nh_.getParam("obstacle_occupancy",	this->obstacle_occupancy_);

	
	ROS_INFO("DynamicGoals frame_id:		    %s", this->frame_id_.c_str());
	ROS_INFO("DynamicGoals obstacles topic:		%s", this->obstacle_topic_.c_str());
	ROS_INFO("DynamicGoals target topic:		%s", this->target_topic_.c_str());
	ROS_INFO("DynamicGoals action server name:	%s", this->actionsrv_.c_str());
	ROS_INFO("DynamicGoals obstacles strength:	%f", this->obstacle_strength_);
	ROS_INFO("DynamicGoals obstacles decay:		%f", this->obstacle_decay_);
	ROS_INFO("DynamicGoals obstacles occupancy:	%f", this->obstacle_occupancy_);

	return true;
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

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
    	this->goal_.target_pose.header.frame_id  = this->frame_id_;
    	this->goal_.target_pose.pose.position.x	 = r*cos(w);
    	this->goal_.target_pose.pose.position.y	 = r*sin(w);
    	this->goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(w);
    	this->goal_.target_pose.header.stamp	 = ros::Time::now();

    	ROS_INFO("New goal at %f [cm] / %f [deg]", r, w*180.0f/M_PI);
    	this->actioncln_->sendGoal(this->goal_);
	}
}

move_base_msgs::MoveBaseGoal DynamicGoals::make_goal(void) {

	float nw, np;
	move_base_msgs::MoveBaseGoal goal;

	nw = this->compute_orientation(this->sector_data_);
	np = this->compute_position_exponential(this->sector_data_, nw);

    goal.target_pose.header.frame_id	= this->frame_id_;
    goal.target_pose.pose.position.x	= np*cos(nw);
    goal.target_pose.pose.position.y	= np*sin(nw);
    goal.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(nw);
    goal.target_pose.header.stamp		= ros::Time::now();
    	
	ROS_INFO("New goal at %f [cm] / %f [deg]", np, nw*180.0f/M_PI);

	return goal;

}

void DynamicGoals::callback(const cnbiros_wheelchair_navigation::SectorGrid& data) {

	float nw, np;

    this->sector_data_ = data;


	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
		
		this->goal_ = this->make_goal();
		this->actioncln_->cancelGoal();
    	this->actioncln_->sendGoal(this->goal_);
	}

	/*
	nw = this->compute_orientation(data);
	//np = this->compute_position(data);
	np = this->compute_position_exponential(data, nw);

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
	
		this->actioncln_->cancelGoal();

    	this->goal_.target_pose.header.frame_id  = this->frame_id_;
    	this->goal_.target_pose.pose.position.x	 = np*cos(nw);
    	this->goal_.target_pose.pose.position.y	 = np*sin(nw);
    	this->goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(nw);
    	this->goal_.target_pose.header.stamp	 = ros::Time::now();

    	ROS_INFO("New goal at %f [cm] / %f [deg]", np, nw*180.0f/M_PI);
    	this->actioncln_->sendGoal(this->goal_);
	}
	*/
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
		
		ROS_INFO("Obstacle at: %f degree", cangle*180.0f/M_PI);	
		clambda = this->obstacle_strength_*exp(-(cradius/this->obstacle_decay_));
		csigma  = std::atan(std::tan(sector_step/2.0f) + 
			           this->obstacle_occupancy_/(this->obstacle_occupancy_ + cradius));
		
		w += clambda*(hangle-cangle)*exp(-(std::pow(hangle - cangle, 2))/(2.0f*pow(csigma, 2)));

    }

	//if (w > M_PI/4.0f) {
	//	w = M_PI/4.0f;
	//} else if(w < -M_PI/4.0f) {
	//	w = -M_PI/4.0f;
	//}
	
	if (w < sector_min_angle) {
		w = sector_min_angle;
	} else if(w > sector_max_angle) {
		w = sector_max_angle;
	}

    return w;

}

float DynamicGoals::compute_position_exponential(const cnbiros_wheelchair_navigation::SectorGrid& data, float wescape) {

	unsigned int idsector;
	float cvalue;
	float tposition;

    float a;
    float MINDIS = 0.0f;
    float MAXDIS = 4.0f;
    float MAXPOS = 2.0f;
    
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
		} else {
			this->obstacle_decay_ = req.value;
			ROS_INFO("Updated obstacle decay to: %f", this->obstacle_decay_);
		}
	} else if( req.name.compare("obstacle_occupancy") == 0 ) {
		if(req.value <= 0.0f) {
			ROS_ERROR("Obstacle occupancy must be > 0.0f");
		} else {
			this->obstacle_occupancy_ = req.value;
			ROS_INFO("Updated obstacle occupancy to: %f", this->obstacle_occupancy_);
		}
	} else {
		ROS_ERROR("'%s' is not a parameter of DynamicGoal node", req.name.c_str());
		res.result = false;
	}

	return res.result;
}

/*
bool DynamicGoals::on_set_obstacle_strength(cnbiros_wheelchair_navigation::ObstacleStrength::Request &req,
									  cnbiros_wheelchair_navigation::ObstacleStrength::Response &res) {

	if(req.strength <= 0.0f) {
		ROS_ERROR("Obstacle strength must be > 0.0f");
		res.result = false;
	} else {
		this->obstacle_strength_ = req.strength;
		ROS_INFO("Updated obstacle strength to: %f", this->obstacle_strength_);
		res.result = true;
	}

	return res.result;
}

bool DynamicGoals::on_set_obstacle_decay(cnbiros_wheelchair_navigation::ObstacleDecay::Request &req,
									  cnbiros_wheelchair_navigation::ObstacleDecay::Response &res) {

	if(req.decay <= 0.0f) {
		ROS_ERROR("Obstacle decay must be > 0.0f");
		res.result = false;
	} else {
		this->obstacle_decay_ = req.decay;
		ROS_INFO("Updated obstacle decay to: %f", this->obstacle_decay_);
		res.result = true;
	}

	return res.result;
}
*/

/*
bool DynamicGoals::compute_velocity(const cnbiros_wheelchair_navigation::SectorGrid& data, float& v) {
    unsigned int nelem;
    float cradius, cangle, cvalue, clambda, csigma; 
    float distance_center, distance_front;

    if( (data.radius.size() != data.angle.size()) ||
	(data.angle.size() != data.value.size()) ) {
	ROS_ERROR("Corrupted PolarGrid message");
	return false;
    }

    v = this->maxvel_;
    nelem = data.radius.size();
    for(auto i=0; i<nelem; i++) {
	cradius = data.radius.at(i);
	cangle  = data.angle.at(i);
	cvalue  = (float)data.value.at(i);
	distance_center = std::abs(std::cos(cangle)*cradius);
	distance_front  = std::sin(cangle)*cradius;
	
	if(distance_center <= this->size_) {
	    distance_front = std::max(distance_front -
			               std::sqrt(std::pow(this->size_,2.0f) -
					         std::pow(distance_center,2.0f)), 0.01f);
	} else {
	    distance_front = distance_front + 
			     std::exp(this->audacity_/this->size_*std::pow((distance_center - this->size_),2.0f))-1.0f;
	}
	v = std::min(v, this->maxvel_*std::exp(-this->lindecay_/distance_front));
    }
    return true;
}
*/
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

				this->goal_ = this->make_goal();
				this->actioncln_->cancelGoal();
    			this->actioncln_->sendGoal(this->goal_);
			} else if (state == actionlib::SimpleClientGoalState::ABORTED) {
				ROS_INFO("Goal aborted. Clearing costmap and forcing new goal");

				if(this->srv_clear_costmap_.call(srvmsg_clearcostmap)) {
					ROS_INFO("Costmap cleared");
				} else {
					ROS_ERROR("Failed to request costmap clearing");
				}

				this->goal_ = this->make_goal();
				this->actioncln_->cancelGoal();
    			this->actioncln_->sendGoal(this->goal_);
			}

		}
		ros::spinOnce();
    }

}

    }
}

#endif
