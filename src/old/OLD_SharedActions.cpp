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
    this->subrep_ = this->nh_.subscribe(this->reptopic_, 1, &SharedActions::on_receive_repellors, this);
    this->subatt_ = this->nh_.subscribe(this->atttopic_, 1, &SharedActions::on_receive_attractors, this);

	// Initialize move base client
    this->actioncln_ = new MoveBaseClient(this->actionsrv_, true);

	// Initialize clearcostmap service
	this->srv_clearcostmap_ = this->nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// Initialize start/stop service server
	this->srv_state_toggle_ = this->private_nh_.advertiseService("toggle_state", &SharedActions::on_request_state_toggle, this);

	this->srv_set_goal_ = this->private_nh_.advertiseService("set_goal", &SharedActions::on_request_set_goal, this);

	// Initializer external goal reset area
	this->pub_ext_goal_area_ = this->private_nh_.advertise<geometry_msgs::PolygonStamped>("/external_goal_area", 1);
	
	// Initializer internal reset area
	this->pub_int_goal_area_ = this->private_nh_.advertise<geometry_msgs::PolygonStamped>("/internal_goal_area", 1);

	// Initialize odom subcriber
	this->subodom_ = this->nh_.subscribe(this->topic_odom_, 1, &SharedActions::on_receive_odometry, this);
	
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
	this->private_nh_.param<float>("repellors_strength", this->repellors_strength_, 0.5f);
	this->private_nh_.param<float>("repellors_decay", this->repellors_decay_, 1.0f);
	//this->private_nh_.param<float>("goal_max_distance", this->goal_max_distance_, 2.0f);
	//this->private_nh_.param<float>("goal_half_position", this->goal_half_position_, 1.5f);
	this->private_nh_.param<float>("goal_linear_max_distance", this->linear_position_max_distance_, 4.0f);
	this->private_nh_.param<float>("goal_linear_slope", this->linear_position_slope_, 1.3f);
	this->private_nh_.param<float>("goal_linear_min_distance", this->linear_position_min_distance_, 1.2f);
	this->private_nh_.param<float>("command_timeout", this->command_timeout_, 8.0f);
	this->private_nh_.param<float>("update_rate", this->update_rate_, 10.0f);
	this->private_nh_.param<bool>("autostart", this->autostart_, false);
	this->private_nh_.param<bool>("on_place", this->on_place_, false);
	this->private_nh_.param<bool>("enable_repellors", this->enable_repellors_, true);
	this->private_nh_.param<bool>("enable_attractors", this->enable_attractors_, true);

	this->private_nh_.param<float>("external_goal_area_radius", this->external_goal_area_radius_, 1.4f);
	this->private_nh_.param<float>("internal_goal_area_radius", this->internal_goal_area_radius_, 0.1f);

	// Dump parameters
	ROS_INFO("SharedActions frame_id:				%s", this->frame_id_.c_str());
	ROS_INFO("SharedActions repellors topic:		%s", this->reptopic_.c_str());
	ROS_INFO("SharedActions attractors topic:		%s", this->atttopic_.c_str());
	ROS_INFO("SharedActions action server name:		%s", this->actionsrv_.c_str());
	ROS_INFO("SharedActions robot circumscribed radius:	%f", this->robot_radius_);
	ROS_INFO("SharedActions repellors strength:		%f", this->repellors_strength_);
	ROS_INFO("SharedActions repellors decay:		%f", this->repellors_decay_);
	//ROS_INFO("SharedActions goal max distance:		%f", this->goal_max_distance_);
	//ROS_INFO("SharedActions goal half position:		%f", this->goal_half_position_);
	ROS_INFO("SharedActions command timeout:		%f", this->command_timeout_);
	ROS_INFO("SharedActions update rate:			%f", this->update_rate_);
	ROS_INFO("SharedActions external goal area radius:		%f", this->external_goal_area_radius_);
	ROS_INFO("SharedActions internal goal area radius:		%f", this->internal_goal_area_radius_);
	ROS_INFO("SharedActions goal linear maxdistance: %f", this->linear_position_max_distance_);
	ROS_INFO("SharedActions goal linear slope:		%f", this->linear_position_slope_);
	ROS_INFO("SharedActions goal linear mindistance: %f", this->linear_position_min_distance_);


	// Create goal area 
	this->external_goal_area_ = this->create_goal_area(this->external_goal_area_radius_);	
	this->internal_goal_area_ = this->create_goal_area(this->internal_goal_area_radius_);	

	// Initialize odometry topic
	this->topic_odom_ = "/odom";

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
	float repangle = 0.0f;
	float attangle = 0.0f;
	float radius = 0.0f;

	
	// Compute orientation for repellors
	if(this->enable_repellors_ == true) {
		repangle  = this->goal_orientation_repellors(this->repellors_data_);
		ROS_DEBUG_NAMED("sharedactions", "Map angle (heading): %f [deg]", repangle*180.0f/M_PI);
	}
	
	// Compute orientation for attractors
	if(this->enable_attractors_ == true) {
		attangle = this->goal_orientation_attractors(this->attractors_data_);
		ROS_DEBUG_NAMED("sharedactions", "Usr angle (heading): %f [deg]", attangle*180.0f/M_PI);
	}
	
	angle = repangle + attangle;

	// Limit the orientation to the front
	angle = this->goal_orientation_limits(angle, -M_PI/2.0f, M_PI/2.0f);	
	
	// Compute position for computed angle (if repellors_data_ is available)
	if(this->is_data_available_ == true && this->on_place_ == false) {
		//radius = this->goal_position_logistic(this->repellors_data_, angle);
		radius = this->goal_position_linear(this->repellors_data_, angle);
	}

	// TMP - DEBUG
	//radius = 0.0f;

	// Rotate angle (standard coordinate frame conventions)	
	angle = M_PI/2.0f + angle;

    // Make the goal given the computed angle and radius
	this->goal_.target_pose.header.frame_id		= this->frame_id_;
    this->goal_.target_pose.pose.position.y		= -radius*cos(angle);
    this->goal_.target_pose.pose.position.x		= radius*sin(angle);
    this->goal_.target_pose.pose.orientation	= tf::createQuaternionMsgFromYaw(angle-M_PI/2.0f);
    this->goal_.target_pose.header.stamp		= ros::Time::now();
  
	geometry_msgs::PointStamped goal_pt;
	goal_pt.header = this->goal_.target_pose.header;
	goal_pt.point  = this->goal_.target_pose.pose.position;
	
	try {
		this->listener_.waitForTransform("odom", "base_link", 
									goal_pt.header.stamp, ros::Duration(10.0) );
		this->listener_.transformPoint("odom", goal_pt, this->goal_position_);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}

	if (radius <= this->external_goal_area_radius_) {
		ROS_INFO("Short goal");
		this->short_goal_ = true;	
	} else {
		this->short_goal_ = false;
	}

	//ROS_DEBUG_NAMED("sharedactions", "New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
	//ROS_INFO("New goal at %f [cm] / %f [deg]", radius, angle*180.0f/M_PI);
}


void SharedActions::SendGoal(void) {

	if(this->actioncln_->isServerConnected() == false) {
    	ROS_WARN("%s action server is disconnected. Nothing to do.", this->actionsrv_.c_str());
    } else {
    	//this->actioncln_->sendGoal(this->goal_);
		//this->actioncln_->sendGoal(this->goal_, boost::bind(&SharedActions::doneCb, this, _1, _2), 
		//							&SharedActions::activeCb, &SharedActions::feedbackCb);
		this->actioncln_->sendGoal(this->goal_, boost::bind(&SharedActions::doneCb, this, _1, _2), 
									MoveBaseClient::SimpleActiveCallback(), boost::bind(&SharedActions::feedbackCb, this, _1));
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
	float current_goal_radius;

    while(this->nh_.ok()) {
		
		ros::spinOnce();
		this->rate_->sleep();

		this->external_goal_area_.header.frame_id = "base_link";
		this->external_goal_area_.header.stamp = ros::Time::now();
		this->internal_goal_area_.header.frame_id = "base_link";
		this->internal_goal_area_.header.stamp = ros::Time::now();
		this->pub_ext_goal_area_.publish(this->external_goal_area_);
		this->pub_int_goal_area_.publish(this->internal_goal_area_);

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
			} else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				this->CancelGoal();
				this->MakeGoal();
				this->SendGoal();
				ROS_INFO("Goal reached. Sent new goal at (%4.2f, %4.2f) [m]", 
						 this->goal_.target_pose.pose.position.x, 
						 this->goal_.target_pose.pose.position.y);
			} 

			// Check if is inside goal area radius
			current_goal_radius = this->short_goal_ ? this->internal_goal_area_radius_ : this->external_goal_area_radius_; 	
			if(this->is_within_goal_area(current_goal_radius)) {
				this->CancelGoal();
				this->MakeGoal();
				this->SendGoal();
				ROS_INFO("Inside goal area (%3.2f). Sent new goal at (%4.2f, %4.2f) [m]",
						 current_goal_radius,
						 this->goal_.target_pose.pose.position.x, 
						 this->goal_.target_pose.pose.position.y);

			}


			//// Force a new goal and cancel the previous ones (semi-autonomous
			//// behaviour).
			//// The new goal is based on the last costmap received and on the
			//// last user command received (if exists)
			//this->MakeGoal();
			//this->CancelGoal();
			//this->SendGoal();
		}
    }

}

bool SharedActions::is_projection_inside(float distance, float angle, float radius) {

	float projection;
	bool result = false;

	projection = std::fabs(distance*cos(angle)); 

	if(projection <= 2.0f*radius)
		result = true;

	return result;
}

float SharedActions::goal_orientation_repellors(ProximitySector& sectors) {

	ProximitySectorConstIt it;
    float sector_step;
    float cdistance, cangle, csigma, clambda;
    float hangle = 0.0f;
    float w = 0.0f;

	float robotsize = 2.0f*(this->robot_radius_);

    sector_step	= sectors.GetStep();

    for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		//cdistance = sectors.GetRadius(it);
		cdistance = sectors.GetRadius(it) - 1.2f;
		cangle  = sectors.GetAngle(it);

		// Check if the repellors is inside the robot radius. If not, skip it.
		if(this->is_projection_inside(cdistance, cangle + M_PI/2.0f, this->robot_radius_) == false)
			continue;

		clambda = this->repellors_strength_*exp(-(cdistance/this->repellors_decay_));
		csigma  = std::atan(std::tan(sector_step/2.0f) + 
			           robotsize/(robotsize + cdistance));
		
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


float SharedActions::goal_position_linear(ProximitySector& sectors, float wescape) {

	ProximitySectorConstIt it;
	float distance;

	float x0, slope;
	float max_goal_distance;

	float goal_distance;

	x0 = this->linear_position_min_distance_;
	slope = this->linear_position_slope_;
	max_goal_distance = this->linear_position_max_distance_;

	try {
		distance = sectors.At(wescape);
	} catch (std::runtime_error e) {
		ROS_ERROR("Error: %s", e.what());
	}

	//ROS_INFO("Distance obstacle at escape direction: %f", distance);
	
	if(std::isinf(distance))
		distance = max_goal_distance;

	goal_distance = (distance)*slope - slope*x0;

	if(goal_distance > max_goal_distance)
		goal_distance = max_goal_distance;


	return goal_distance;


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
	
	//if(std::fabs(config.param_goal_max_distance - this->goal_max_distance_) > 0.00001f) {
	//	this->goal_max_distance_ = config.param_goal_max_distance;
	//	ROS_WARN("Updated goal max distance to %f", this->goal_max_distance_);
	//}
	//
	//if(std::fabs(config.param_goal_half_position - this->goal_half_position_) > 0.00001f) {
	//	this->goal_half_position_ = config.param_goal_half_position;
	//	ROS_WARN("Updated goal half position to %f", this->goal_half_position_);
	//}
	
	if(std::fabs(config.param_linear_position_max_distance - this->linear_position_max_distance_) > 0.00001f) {
		this->linear_position_max_distance_ = config.param_linear_position_max_distance;
		ROS_WARN("Updated max distance goal (linear) to %f", this->linear_position_max_distance_);
	}
	
	if(std::fabs(config.param_linear_position_slope - this->linear_position_slope_) > 0.00001f) {
		this->linear_position_slope_ = config.param_linear_position_slope;
		ROS_WARN("Updated goal slope (linear) to %f", this->linear_position_slope_);
	}
	
	if(std::fabs(config.param_linear_position_min_distance - this->linear_position_min_distance_) > 0.00001f) {
		this->linear_position_min_distance_ = config.param_linear_position_min_distance;
		ROS_WARN("Updated min distance goal (linear) to %f", this->linear_position_min_distance_);
	}
	
	if(std::fabs(config.param_external_goal_area_radius - this->external_goal_area_radius_) > 0.00001f) {
		this->external_goal_area_radius_ = config.param_external_goal_area_radius;
		this->external_goal_area_ = this->create_goal_area(this->external_goal_area_radius_);
		ROS_WARN("Updated external goal area radius to %f", this->external_goal_area_radius_);
	}
	
	if(std::fabs(config.param_internal_goal_area_radius - this->internal_goal_area_radius_) > 0.00001f) {
		this->internal_goal_area_radius_ = config.param_internal_goal_area_radius;
		this->internal_goal_area_ = this->create_goal_area(this->internal_goal_area_radius_);
		ROS_WARN("Updated internal goal area radius to %f", this->internal_goal_area_radius_);
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
	this->CancelGoal();

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
		this->CancelGoal();
		this->MakeGoal();
		this->SendGoal();
		ROS_INFO("Sent new goal at (%4.2f, %4.2f) [m]", 
				  this->goal_.target_pose.pose.position.x, 
				  this->goal_.target_pose.pose.position.y);
	}
	res.success = true;

	return res.success;
}

bool SharedActions::on_request_set_goal(std_srvs::Empty::Request& req,
										std_srvs::Empty::Response& res) {

	bool result = false;

	if(this->IsRunning() == true) {
		this->CancelGoal();
		this->MakeGoal();
		this->SendGoal();
		ROS_INFO("Sent new goal at (%4.2f, %4.2f) [m]", 
				  this->goal_.target_pose.pose.position.x, 
				  this->goal_.target_pose.pose.position.y);
		result = true;
	}
	

	return result;
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


geometry_msgs::PolygonStamped SharedActions::create_goal_area(float radius) {

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

void SharedActions::on_receive_odometry(const nav_msgs::Odometry& data) {
	this->odom_ = data;
}

bool SharedActions::is_within_goal_area(float goal_area_radius) {

	float distance;
	bool result = false;

	geometry_msgs::PointStamped	odom;
	geometry_msgs::PointStamped	goal, goal_tf;

	odom.header = this->odom_.header;
	odom.point = this->odom_.pose.pose.position;
	//goal.header = this->goal_.target_pose.header;
	//goal.header.stamp = odom.header.stamp;
	//goal.point = this->goal_.target_pose.pose.position;

	//try {
	//	this->listener_.waitForTransform("odom", "base_link", 
	//								ros::Time(0), ros::Duration(10.0) );
	//	this->listener_.transformPoint("odom", goal, goal_tf);
	//} catch (tf::TransformException &ex) {
	//	ROS_ERROR("%s", ex.what());
	//	return false;
	//}
	

	distance = std::hypot((odom.point.x - this->goal_position_.point.x), (odom.point.y - this->goal_position_.point.y));

	if(distance <= goal_area_radius)
		result = true;

	ROS_INFO("Current position: (%3.2f, %3.2f)", odom.point.x, odom.point.y);
	ROS_INFO("Target position:  (%3.2f, %3.2f)", this->goal_position_.point.x, this->goal_position_.point.y);
	ROS_INFO("Distance: %f", distance);

	return result;
}

void SharedActions::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
}

void SharedActions::activeCb(void) {}

void SharedActions::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {

	float distance;
	geometry_msgs::Point position;
	geometry_msgs::Point goal;

	position = feedback->base_position.pose.position;
	goal = this->goal_.target_pose.pose.position;


	distance = std::hypot((goal.x - position.x), (goal.y - position.y));
	
	//ROS_INFO("Distance feedback: %f", distance);

}

/*
float SharedActions::goal_position_logistic(ProximitySector& sectors, float wescape) {

	//float cvalue = std::numeric_limits<float>::infinity();
	ProximitySectorConstIt it;
	float goaldistance;
	std::vector<float> colliding_distances;
	std::vector<float> colliding_angles;

    float MaxPosition = this->goal_max_distance_;
	float MinPosition = SHAREDACTIONS_LOGISTIC_MINDISTANCE;
	float MinDistance = this->robot_radius_;
	float HalfPosition = this->goal_half_position_;
	float Slope;

	float cangle, cdistance, cprojection;
	float closest_distance = MaxPosition;
	float closest_angle = 0.0f;
	float cvalue;

	for(it=sectors.Begin(); it!=sectors.End(); ++it) {

		if(std::isinf((*it)) == true)
		    continue;

		cdistance	= sectors.GetRadius(it);
		cangle		= sectors.GetAngle(it);
		cprojection = std::fabs(cdistance*cos(cangle)); 
		
		//// Check if the projection is inside the radius. In that case, store the
		//// distance
		//if(cprojection <= this->robot_radius_) {
		//	colliding_distances.push_back(cdistance);
		//	colliding_angles.push_back(cangle-M_PI/2.0f);
		//}

		// Check if the repellors is inside the robot radius. If not, skip it.
		if(this->is_projection_inside(cdistance, cangle + M_PI/2.0f, this->robot_radius_) == false)
			continue;
			
		colliding_distances.push_back(cdistance);
		colliding_angles.push_back(cangle);
	}

	auto i = 0;
	for(auto itd=colliding_distances.begin(); itd!=colliding_distances.end(); ++itd) {
		if(closest_distance < (*itd)) {
			closest_distance = (*itd);
			closest_angle    = colliding_angles.at(i);
		}
		i++;
	}
	ROS_INFO("Closest obstacle at %f [m] and %f [deg]", closest_distance, closest_angle*180.0f/M_PI);

	// Using the closest distance in the logistic regression
	cvalue = closest_distance;

	//for(auto it=sectors.Begin(); it!=sectors.End(); ++it)
	//	cvalue = std::min(cvalue, (*it));

	//try {
	//	cvalue = sectors.At(wescape);
	//} catch (std::runtime_error e) {
	//	ROS_ERROR("Error: %s", e.what());
	//}
	
	Slope = -std::log((MaxPosition/MinPosition) - 1.0f)*(1.0f/(MinDistance - HalfPosition));
	
	if(std::isinf(cvalue)) {
		goaldistance = MaxPosition;
	} else {
		goaldistance = MaxPosition/(1.0f + std::exp(-Slope*(cvalue-HalfPosition)));
	}

	return goaldistance;

}
*/

    }
}

#endif
