#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_HPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include "cnbiros_wheelchair_navigation/SectorGrid.h"

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class DynamicGoals {
    
    public:
		DynamicGoals(void);
		virtual ~DynamicGoals(void);

		void WaitForServer(void);

		bool configure(void);

    protected:
		virtual void callback(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_orientation(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_position(const cnbiros_wheelchair_navigation::SectorGrid& data);
	

    protected:
	ros::NodeHandle nh_;
	ros::NodeHandle	private_nh_;

	std::string		obstacle_topic_;
	ros::Subscriber	subobstacles_;
	
	std::string		target_topic_;
	ros::Subscriber	subtargets_;

	ros::ServiceClient  rossrv_;
	
	std::string		actionsrv_;
	MoveBaseClient*	actioncln_;

	float			obstacle_strength_;
	float			obstacle_decay_;

	std::string		frame_id_;
	move_base_msgs::MoveBaseGoal	goal_;
	cnbiros_wheelchair_navigation::SectorGrid sector_data_;

	// tmp
	float size_;

};

    }
}

#endif
