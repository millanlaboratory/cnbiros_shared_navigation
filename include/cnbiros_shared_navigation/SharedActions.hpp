#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include "cnbiros_shared_navigation/DynamicGoalsParameters.h"
#include <dynamic_reconfigure/server.h>

// Package include
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
#include "cnbiros_shared_navigation/SharedActionsConfig.h"

#define EPSILON	0.00001

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SharedActions {
    
    public:
		SharedActions(void);
		virtual ~SharedActions(void);

		void WaitForServer(void);

		void MakeGoal(void);
		void MakeGoal(float radius, float angle);
		void SendGoal(void);
		void CancelServerGoal(void);

		void Start(void);
		void Run(void);
		bool configure(void);

    protected:
		void reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, uint32_t level);

		void on_receive_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_receive_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data);

		float goal_orientation_repellors(ProximitySector& data);
		float goal_orientation_attractors(ProximitySector& data);

		float goal_position_exponential(ProximitySector& data, float w);
		float goal_position_logistic(ProximitySector& data, float w);


		void on_reset_command_user(const ros::TimerEvent& event);
		
		//bool on_set_parameters(cnbiros_shared_navigation::DynamicGoalsParameters::Request& req,
		//					   cnbiros_shared_navigation::DynamicGoalsParameters::Response& res);


		//unsigned int angle2sector(float angle, float min_angle, float max_angle,
		//						  float step_angle, unsigned int nsectors);

	private:
		void init_command_timer(float timeout);
		void init_update_rate(float rate);
		void change_update_rate(float rate);

    protected:
		ros::NodeHandle nh_;
		ros::NodeHandle	private_nh_;

		ros::Subscriber	subrep_;
		ros::Subscriber	subatt_;
		
		std::string		reptopic_;
		std::string		atttopic_;

		//ros::ServiceServer  srv_set_parameters_;
		ros::ServiceClient  srv_clear_costmap_;
		
		std::string		actionsrv_;
		MoveBaseClient*	actioncln_;


		std::string		frame_id_;
		move_base_msgs::MoveBaseGoal	goal_;

		ProximitySector repellors_data_;
		ProximitySector attractors_data_;
		
		float	repellors_strength_;
		float	repellors_decay_;
		float	repellors_occupancy_;


		float		command_timeout_;


		float max_goal_distance_;
		float slope_distance_;

		ros::Timer	command_timer_;
		

		dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig> cfgserver;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig>::CallbackType f;

		float update_rate_;
		ros::Rate* rate_;
};

    }
}

#endif
