#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_HPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include "cnbiros_wheelchair_navigation/SectorGrid.h"
//#include "cnbiros_wheelchair_navigation/ObstacleStrength.h"
//#include "cnbiros_wheelchair_navigation/ObstacleDecay.h"
#include "cnbiros_wheelchair_navigation/DynamicGoalsParameters.h"

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class DynamicGoals {
    
    public:
		DynamicGoals(void);
		virtual ~DynamicGoals(void);

		void WaitForServer(void);

		void Start(void);
		void Run(void);
		bool configure(void);

    protected:
		move_base_msgs::MoveBaseGoal make_goal(void);

		virtual void callback(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_orientation(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_position_linear(const cnbiros_wheelchair_navigation::SectorGrid& data, float w);
		virtual float compute_position_exponential(const cnbiros_wheelchair_navigation::SectorGrid& data, float w);
		virtual float compute_position(const cnbiros_wheelchair_navigation::SectorGrid& data);

		/*
		bool on_set_obstacle_strength(cnbiros_wheelchair_navigation::ObstacleStrength::Request &req,
									  cnbiros_wheelchair_navigation::ObstacleStrength::Response &res);
		bool on_set_obstacle_decay(cnbiros_wheelchair_navigation::ObstacleDecay::Request &req,
								   cnbiros_wheelchair_navigation::ObstacleDecay::Response &res);
*/

		bool on_set_parameters(cnbiros_wheelchair_navigation::DynamicGoalsParameters::Request& req,
							   cnbiros_wheelchair_navigation::DynamicGoalsParameters::Response& res);

		unsigned int angle2sector(float angle, float min_angle, float max_angle,
								  float step_angle, unsigned int nsectors);

    protected:
		ros::NodeHandle nh_;
		ros::NodeHandle	private_nh_;

		std::string		obstacle_topic_;
		ros::Subscriber	subobstacles_;
		
		std::string		target_topic_;
		ros::Subscriber	subtargets_;

		//ros::ServiceServer  srv_obstacle_strength_;
		//ros::ServiceServer  srv_obstacle_decay_;
		ros::ServiceServer  srv_set_parameters_;
		ros::ServiceClient  srv_clear_costmap_;
		
		std::string		actionsrv_;
		MoveBaseClient*	actioncln_;

		float			obstacle_strength_;
		float			obstacle_decay_;
		float			obstacle_occupancy_;

		std::string		frame_id_;
		move_base_msgs::MoveBaseGoal	goal_;
		cnbiros_wheelchair_navigation::SectorGrid sector_data_;

};

    }
}

#endif
