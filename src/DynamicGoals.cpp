#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_CPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_CPP

#include "cnbiros_wheelchair_navigation/DynamicGoals.hpp"

namespace cnbiros {
    namespace navigation {

DynamicGoals::DynamicGoals(void) : private_nh_("~") {

    this->actioncln_ = nullptr;
    
	// Configure node
	this->configure();

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

	this->obstacle_strength_ = 4.0f;
	this->obstacle_decay_    = 2.0f;
	this->obstacle_topic_	 = "/sectorgrid";
	this->target_topic_   	 = "/tmpname";
	this->actionsrv_	  	 = "move_base";
	this->frame_id_		  	 = "base_link";

	// Getting parameters
	this->private_nh_.getParam("obstacles",			this->obstacle_topic_);
	this->private_nh_.getParam("targets",			this->target_topic_);
	this->private_nh_.getParam("action_server",		this->actionsrv_);
	this->private_nh_.getParam("frame_id",			this->frame_id_);
	this->private_nh_.getParam("obstacle_strength", this->obstacle_strength_);
	this->private_nh_.getParam("obstacle_decay",	this->obstacle_decay_);

	
	// Compute size
	this->size_ = 1.0f;
	
	
	ROS_INFO("DynamicGoals frame_id:		    %s", this->frame_id_.c_str());
	ROS_INFO("DynamicGoals obstacles topic:		%s", this->obstacle_topic_.c_str());
	ROS_INFO("DynamicGoals target topic:		%s", this->target_topic_.c_str());
	ROS_INFO("DynamicGoals action server name:	%s", this->actionsrv_.c_str());
	ROS_INFO("DynamicGoals obstacles strength:	%f", this->obstacle_strength_);
	ROS_INFO("DynamicGoals obstacles decay:		%f", this->obstacle_decay_);




	return true;
}

void DynamicGoals::WaitForServer(void) {

    while(!(this->actioncln_->waitForServer(ros::Duration(1.0f)))) {
		ROS_INFO_THROTTLE(5.0, "Waiting for %s action server to come up", this->actionsrv_.c_str());
	}
    ROS_INFO("%s action server connected", this->actionsrv_.c_str());
}

void DynamicGoals::callback(const cnbiros_wheelchair_navigation::SectorGrid& data) {

	float nw, np;

    this->sector_data_ = data;

	nw = this->compute_orientation(data);
	np = this->compute_position(data);

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
}

float DynamicGoals::compute_orientation(const cnbiros_wheelchair_navigation::SectorGrid& data) {

    unsigned int i;
    unsigned int nsectors;
    float sector_step, sector_min_angle;
    float cradius, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

    sector_step	     = data.step;
    sector_min_angle = data.min_angle;
    nsectors	     = data.nsectors;

    i = 0;
    for(auto it=data.values.begin(); it!=data.values.end(); ++it) {
	i++;

	if(std::isinf((*it)) == true)
	    continue;

	cradius = (*it);
	cangle  = sector_min_angle + sector_step*((i-1) + 0.5f);
	
	ROS_INFO("Obstacle at: %f degree", cangle*180.0f/M_PI);	
	clambda = this->obstacle_strength_*exp(-(cradius/this->obstacle_decay_))/(float)nsectors;
	csigma  = std::atan(std::tan(sector_step/2.0f) + 
		           this->size_/(this->size_ + cradius));
	
	w += clambda*(hangle-cangle)*exp(-(std::pow(hangle - cangle, 2))/(2.0f*pow(csigma, 2)));

    }

    return w;

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
/*
void DynamicGoals::Run(void) {
   
    float w, p;
    ros::Rate r(20);

    geometry_msgs::PoseStamped tmppose;

    while(this->nh_.ok()) {
	
	ros::spinOnce();
	r.sleep();
	
	
	if(this->actioncln_->isServerConnected() == false) {
    	    ROS_WARN("%s action server is disconnected. Nothing to do.", this->server_name_.c_str());
	    continue;
    	}

	if(this->new_costmap_ == false) 
	    continue;

	// Initialize new orientation goal
	w = this->compute_orientation(this->rep_data_);
    	ROS_INFO("current orientation goal: %f [deg]", w*180.0f/M_PI);
	
	p = this->compute_position(this->rep_data_);
    	ROS_INFO("current position goal: %f [cm]", p);
	
	// Tmp pose message
	tmppose.header.stamp = ros::Time::now();
	tmppose.header.frame_id = "base_link";
	tmppose.pose.position.x = p*cos(w);
	tmppose.pose.position.y = p*sin(w);
	tmppose.pose.position.z = 0.0f;
	tmppose.pose.orientation = tf::createQuaternionMsgFromYaw(w);


	this->tmppub_.publish(tmppose);
	//v = 0.0f;
	
	//if(this->compute_orientation(this->rep_data_, w) == false) {
    	//    ROS_ERROR("Cannot compute orientation from incoming message");
    	//    continue;
    	//}
	
	//if(this->compute_velocity(this->rep_data_, v) == false) {
    	//    ROS_ERROR("Cannot compute velocity from incoming message");
    	//    continue;
    	//}


	ROS_INFO("Canceling all current goals");
    	this->actioncln_->cancelAllGoals();

    	//ROS_INFO("current velocity goal: %f", v);
    	this->current_goal_.target_pose.header.frame_id = "base_link";
    	this->current_goal_.target_pose.pose.position.x = p*cos(w);
    	this->current_goal_.target_pose.pose.position.y = p*sin(w);
    	this->current_goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(w);

    	this->current_goal_.target_pose.header.stamp = ros::Time::now();
    	ROS_INFO("Sending new goal");
    	this->actioncln_->sendGoal(this->current_goal_);

    }

}
*/

    }
}

#endif
