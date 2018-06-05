#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDDYNAMICS_HPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDDYNAMICS_HPP

// System includes
#include <exception>

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>

// Package includes
#include "cnbiros_shared_navigation/ProximityGrid.hpp"
#include "cnbiros_shared_navigation/ProximityGridMsg.h"
#include "cnbiros_shared_navigation/ProximityGridConverter.hpp"
#include "cnbiros_shared_navigation/SharedDynamicsConfig.h"

namespace cnbiros {
	namespace navigation {

typedef dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedDynamicsConfig>	DynamicReconfig;

class SharedDynamics {

	public:
		SharedDynamics(void);
		virtual ~SharedDynamics(void);
	
		bool configure(void);
		void Run(void);


		void MakeVelocity(void);
		void SendVelocity(void);
		void SendVelocity(float x, float w);
		void ResetVelocity(void);

	private:

		float get_angular_velocity_repellors(ProximityGrid& data);
		float get_angular_velocity_attractors(float angle);
		float get_linear_velocity_repellors(ProximityGrid& data);
		bool  is_projection_inside(float distance, float angle, float width);

		// Callbacks
		void on_received_repellors(const cnbiros_shared_navigation::ProximityGridMsg& data);
		void on_received_attractors(const cnbiros_shared_navigation::ProximityGridMsg& data);

		void on_publish_velocity(const ros::TimerEvent& event);
		void on_target_elapsed(const ros::TimerEvent& event);
		void reconfigure_callback(cnbiros_shared_navigation::SharedDynamicsConfig &config, uint32_t level);

		// Utilities
		float	rad2deg(float radians);
		bool	update_if_different(const float& first, float& second, float epsilon = 0.00001f);
		float	limit_range_value(float x, float minx, float maxx);
		float	scale_range_value(float x, float min_abs_x, float max_abs_x, float min_abs_y, float max_abs_y);

		void	init_update_rate(float rate);

	private:
		// Node handles
		ros::NodeHandle nh_;
		ros::NodeHandle	p_nh_;

		// Topics
		std::string t_repellors_;
		std::string t_attractors_;
		std::string	t_velocity_;

		// Subscribers
		ros::Subscriber s_repellors_;
		ros::Subscriber s_attractors_;

		// Publishers
		ros::Publisher	p_velocity_;

		// Proximity grid
		ProximityGrid	pr_repellors_;
		//ProximityGrid   pr_attractors_;

		// Dynamic reconfigure
		DynamicReconfig					n_dynreconfig_server_;
		DynamicReconfig::CallbackType	n_dynreconfig_function_;

		// Robot
		std::string		base_frame_;
		float			size_right_;
		float			size_left_;
		float			safe_distance_front_;
		float			safe_distance_lateral_;

		// General boolean states
		bool	is_data_available_;
		
		// General node variables
		bool		n_autostart_;
		bool		n_enable_repellors_;
		bool		n_enable_attractors_;
		float		n_update_rate_;
		ros::Rate* 	n_rate_;
		ros::Timer  publish_timer_;
		ros::Timer	target_timer_;
		float		publish_frequency_;

		// Velocity Dynamics
		float	dyn_angular_velocity_min_;
		float	dyn_angular_velocity_max_;
		float	dyn_linear_velocity_min_;
		float	dyn_linear_velocity_max_;
		
		float	dyn_angular_repellors_strength_;
		float	dyn_angular_attractors_strength_;
		float	dyn_angular_repellors_decay_;
		float	dyn_linear_velocity_decay_;
		
		float	target_;
		float	target_duration_;
		geometry_msgs::Twist		velocity_;
};
		
	}
}

#endif
