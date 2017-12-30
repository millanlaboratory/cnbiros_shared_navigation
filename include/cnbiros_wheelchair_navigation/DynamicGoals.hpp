#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_HPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "cnbiros_wheelchair_navigation/PolarGrid.h"

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class DynamicGoals {
    
    public:
	DynamicGoals(void);
	virtual ~DynamicGoals(void);

	void WaitForServer(void);

	void Run(void);
    protected:
	virtual void callback(const cnbiros_wheelchair_navigation::PolarGrid& data_in);
	virtual bool compute_orientation(const cnbiros_wheelchair_navigation::PolarGrid& data, float& w);

    protected:
	ros::NodeHandle nh_;
	ros::NodeHandle	private_nh_;

	ros::Subscriber	    subrep_;
	ros::Subscriber	    subatt_;

	std::string	    rtopic_;
	std::string	    atopic_;
	std::string	    server_name_;

	MoveBaseClient*			client_;
	move_base_msgs::MoveBaseGoal	current_goal_;
	float				server_timeout_;

	float beta1_;
	float beta2_;
	float sector_;
	float size_;

	bool new_costmap_;
	cnbiros_wheelchair_navigation::PolarGrid rep_data_;


};

    }
}

#endif
