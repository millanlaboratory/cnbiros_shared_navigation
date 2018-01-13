#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_HPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "cnbiros_wheelchair_navigation/SectorGrid.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

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
	virtual void callback(const cnbiros_wheelchair_navigation::SectorGrid& data_in);
	virtual float compute_orientation(const cnbiros_wheelchair_navigation::SectorGrid& data);
	virtual float compute_position(const cnbiros_wheelchair_navigation::SectorGrid& data);
	
	//virtual bool compute_velocity(const cnbiros_wheelchair_navigation::SectorGrid& data, float& v);

    protected:
	ros::NodeHandle nh_;
	ros::NodeHandle	private_nh_;

	ros::Subscriber	    subrep_;
	ros::Subscriber	    subatt_;

	ros::Publisher	    tmppub_;
	ros::ServiceClient  rossrv_;
	
	    std::string	    rtopic_;
	std::string	    atopic_;
	std::string	    server_name_;

	MoveBaseClient*			client_;
	move_base_msgs::MoveBaseGoal	current_goal_;
	float				server_timeout_;

	float beta1_;
	float beta2_;
	float size_;

	float maxvel_;
	float audacity_;
	float lindecay_;

	bool new_costmap_;
	cnbiros_wheelchair_navigation::SectorGrid rep_data_;


};

    }
}

#endif
