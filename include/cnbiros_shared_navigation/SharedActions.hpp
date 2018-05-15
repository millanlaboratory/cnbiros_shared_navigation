#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP

// System includes
#include <exception>

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
#include "cnbiros_shared_navigation/SharedActionsConfig.h"


namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig>	DynamicReconfig;

class SharedActions {
    
	public:
		enum class State {Quit, Stopped, Paused, Running};

    public:
		SharedActions(void);
		virtual ~SharedActions(void);

		bool IsConnected(void);
		void MakeGoal(void);
		void SendGoal(void);
		void CancelGoal(void);

		bool configure(void);

		void Run(void);
		
		bool IsRunning(void);
		void Start(void);
		void Stop(void);
		void Pause(void);
		void Quit(void);
		State GetState(void);



    protected:

		float get_angle_on_repellors(ProximitySector& data);
		float get_angle_on_attractors(ProximitySector& data);
		float get_angle_limits(float angle, float minangle, float maxangle);	
		float get_distance(ProximitySector& data, float angle);

		void reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, uint32_t level);

		bool is_within_reset_area(float radius);
		bool update_if_different(const float& first, float& second, float epsilon = 0.00001f);
		
		bool is_projection_inside(float distance, float angle, float radius);

	private:
		void init_command_timer(float timeout);
		void init_update_rate(float rate);
		void change_update_rate(float rate);
		
		void on_received_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_received_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_received_odometry(const nav_msgs::Odometry& data);
		//void on_received_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
		
		void on_reset_command_user(const ros::TimerEvent& event);
		
		bool on_requested_state_toggle(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool on_requested_set_goal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

		geometry_msgs::PolygonStamped get_reset_area(float radius);

		float rad2deg(float radians);
		float deg2rad(float degrees);

    protected:
		// Node handles
		ros::NodeHandle nh_;
		ros::NodeHandle	p_nh_;

		// Topics
		std::string t_repellors_;
		std::string t_attractors_;
		std::string	t_odom_;

		// Visualization topics
		std::string t_reset_area_;

		// Publishers
		ros::Publisher	p_reset_area_;

		// Subscribers
		ros::Subscriber s_repellors_;
		ros::Subscriber s_attractors_;
		ros::Subscriber s_odom_;

		// Messages
		nav_msgs::Odometry				m_odom_;
		move_base_msgs::MoveBaseGoal	m_goal_;
		geometry_msgs::PointStamped		m_goal_point_;
		geometry_msgs::PolygonStamped	m_reset_area_;

		// Actions
		std::string			a_server_name_;
		MoveBaseClient*		a_client_;

		// Services
		ros::ServiceServer	srv_state_toggle_;
		ros::ServiceServer	srv_set_goal_;

		// Proximity sectors
		ProximitySector	pr_repellors_;
		ProximitySector pr_attractors_;
		
		// Robot
		std::string		robot_base_frame_;
		float			robot_size_;

		// Node variables
		bool			n_autostart_;
		bool			n_enable_repellors_;
		bool			n_enable_attractors_;
		float			n_update_rate_;
		float			n_command_timeout_;
		ros::Timer		n_command_timer_;
		ros::Rate* 		n_rate_;
		State			n_state_;

		tf::TransformListener			n_listener_;
		DynamicReconfig					n_dynreconfig_server_;
		DynamicReconfig::CallbackType	n_dynreconfig_function_;

		// Shared action variables
		float	sa_repellors_strength_;
		float	sa_repellors_decay_;
		float	sa_repellors_occupancy_;
		float	sa_distance_slope_;
		float	sa_distance_max_;
		float	sa_distance_zero_;
		float	sa_reset_radius_;

		// General boolean states
		bool	is_data_available_;
		bool	is_short_goal_;
		bool	is_running_;

		//std::string		reptopic_;
		//std::string		atttopic_;
		//ros::Subscriber	subrep_;
		//ros::Subscriber	subatt_;
		//ros::Publisher	pub_ext_goal_area_;
		//ros::Publisher	pub_int_goal_area_;
		//ros::Subscriber subodom_;
		//std::string		topic_odom_;
		//nav_msgs::Odometry	odom_;
		

		//std::string		actionsrv_;
		//MoveBaseClient*	actioncln_;

		//ros::ServiceClient  srv_clearcostmap_;
		//ros::ServiceServer	srv_state_toggle_;
		//ros::ServiceServer	srv_set_goal_;
		//
		//move_base_msgs::MoveBaseGoal	goal_;
		//geometry_msgs::PointStamped		goal_position_;

		//bool		is_running_;
		//bool		autostart_;
		//bool		on_place_;
		//bool		enable_repellors_;
		//bool		enable_attractors_;

		//std::string	frame_id_;
		//float		robot_radius_;
		//float		repellors_strength_;
		//float		repellors_decay_;
		//float		goal_max_distance_;
		//float		goal_half_position_;
		//float		command_timeout_;
		//bool		is_data_available_;

		//float		linear_position_max_distance_;
		//float		linear_position_slope_;
		//float		linear_position_min_distance_;

		//float		external_goal_area_radius_;
		//float		internal_goal_area_radius_;
		//bool		short_goal_;
		//geometry_msgs::PolygonStamped	external_goal_area_;
		//geometry_msgs::PolygonStamped	internal_goal_area_;
		
		//ProximitySector repellors_data_;
		//ProximitySector attractors_data_;

		//ros::Timer	command_timer_;

		//dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig> cfgserver_;
		//dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig>::CallbackType f_;

		//float update_rate_;
		//ros::Rate* rate_;

		//tf::TransformListener	listener_;
};

    }
}

#endif
