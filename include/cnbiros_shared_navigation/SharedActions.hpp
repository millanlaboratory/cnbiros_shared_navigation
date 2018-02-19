#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP

// System includes
#include <exception>

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
#include "cnbiros_shared_navigation/SharedActionsConfig.h"

#define SHAREDACTIONS_LOGISTIC_MINDISTANCE		0.01f
#define SHAREDACTIONS_LOGISTIC_MEDIUMDISTANCE	2.0f

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SharedActions {
    
    public:
		SharedActions(void);
		virtual ~SharedActions(void);

		void WaitForServer(void);

		void MakeGoal(void);
		void SendGoal(void);
		void CancelGoal(void);

		void Start(void);
		void Run(void);
		bool configure(void);

    protected:
		float goal_orientation_repellors(ProximitySector& data);
		float goal_orientation_attractors(ProximitySector& data);
		float goal_orientation_limits(float angle, float minangle, float maxangle);	

		float goal_position_exponential(ProximitySector& data, float w);
		float goal_position_logistic(ProximitySector& data, float w);

	private:
		void init_command_timer(float timeout);
		void init_update_rate(float rate);
		void change_update_rate(float rate);
		
		void on_receive_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_receive_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		
		void reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, uint32_t level);
		
		void on_reset_command_user(const ros::TimerEvent& event);

    protected:
		ros::NodeHandle nh_;
		ros::NodeHandle	private_nh_;

		std::string		reptopic_;
		std::string		atttopic_;
		ros::Subscriber	subrep_;
		ros::Subscriber	subatt_;
		
		std::string		actionsrv_;
		MoveBaseClient*	actioncln_;

		ros::ServiceClient  srv_clearcostmap_;
		
		move_base_msgs::MoveBaseGoal	goal_;

		std::string	frame_id_;
		float		repellors_strength_;
		float		repellors_decay_;
		float		repellors_occupancy_;
		float		goal_max_distance_;
		float		goal_decay_distance_;
		float		command_timeout_;
		
		ProximitySector repellors_data_;
		ProximitySector attractors_data_;

		ros::Timer	command_timer_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig>::CallbackType f_;

		float update_rate_;
		ros::Rate* rate_;
};

    }
}

#endif
