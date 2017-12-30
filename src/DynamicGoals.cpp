#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_CPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_CPP

#include "cnbiros_wheelchair_navigation/DynamicGoals.hpp"

namespace cnbiros {
    namespace navigation {

DynamicGoals::DynamicGoals(void) : private_nh_("~") {

    this->client_ = nullptr;
    
    this->rtopic_ = "/polar";
    this->atopic_ = "/attractors";
    this->server_name_	    = "move_base";
    this->server_timeout_   = 1.0f;

    this->beta1_ = 150.0f;
    this->beta2_ = 1.0f;
    this->sector_ = 0.05f;
    this->size_  = 1.0f;

    this->subrep_ = this->nh_.subscribe(this->rtopic_, 50, &DynamicGoals::callback, this);
    this->subatt_ = this->nh_.subscribe(this->atopic_, 50, &DynamicGoals::callback, this);

    this->client_ = new MoveBaseClient(this->server_name_, true);
}

DynamicGoals::~DynamicGoals(void) {
    if(this->client_ == nullptr)
	delete this->client_;
}

void DynamicGoals::WaitForServer(void) {

    while(!(this->client_->waitForServer(ros::Duration(this->server_timeout_)))) {
	ROS_INFO_THROTTLE(5.0, "Waiting for %s action server to come up", 
			       this->server_name_.c_str());
	}
    ROS_INFO("%s action server connected", this->server_name_.c_str());

}

void DynamicGoals::callback(const cnbiros_wheelchair_navigation::PolarGrid& data_in) {

    float w = 0.0f;

    this->rep_data_ = data_in;
    this->new_costmap_ = true;

}

bool DynamicGoals::compute_orientation(const cnbiros_wheelchair_navigation::PolarGrid& data, float& w) {

    unsigned int nelem;
    float cradius, cangle, cvalue, clambda, csigma; 

    if( (data.radius.size() != data.angle.size()) ||
	(data.angle.size() != data.value.size()) ) {
	ROS_ERROR("Corrupted PolarGrid message");
	return false;
    }

    w = 0.0f;
    nelem = data.radius.size();
    for(auto i=0; i<nelem; i++) {
	
	cradius = data.radius.at(i);
	cangle  = data.angle.at(i);
	cvalue  = (float)data.value.at(i);
	
	clambda = cvalue*this->beta1_*exp(-(cradius/this->beta2_))/(float)nelem;
	csigma  = std::atan(std::tan(this->sector_/2.0f) + 
		           this->size_/(this->size_ + cradius));
	
	w += clambda*(-cangle)*exp(-std::pow(-cangle, 2)/(2.0f*pow(csigma, 2)));

    }

    return true;

}

void DynamicGoals::Run(void) {
   
    float w;
    ros::Rate r(20);
    while(this->nh_.ok()) {
	
	ros::spinOnce();
	r.sleep();
	
	
	if(this->client_->isServerConnected() == false) {
    	    ROS_WARN("%s action server is disconnected. Nothing to do.", this->server_name_.c_str());
	    continue;
    	}

	// Initialize new orientation goal
	w = 0.0f;
	
	if(this->compute_orientation(this->rep_data_, w) == false) {
    	    ROS_ERROR("Cannot compute orientation from incoming message");
    	    continue;
    	}


	ROS_INFO("Canceling all current goals");
    	this->client_->cancelAllGoals();

    	ROS_INFO("current orientation goal: %f", w);

    	this->current_goal_.target_pose.header.frame_id = "base_link";
    	this->current_goal_.target_pose.pose.position.x = cos(w);
    	this->current_goal_.target_pose.pose.position.y = sin(w);
    	this->current_goal_.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(w);

    	this->current_goal_.target_pose.header.stamp = ros::Time::now();
    	ROS_INFO("Sending new goal");
    	this->client_->sendGoal(this->current_goal_);

    }

}


    }
}

#endif
