#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDDYNAMICS_HPP

// System includes
#include <exception>

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
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

		// Robot
		void get_robot_dimension(float* lenght, float* width, geometry_msgs::Polygon polygon);

		// Velocity Dynamics
		float get_angular_velocity_repellors(ProximitySector& data);
		float get_angular_velocity_max(ProximitySector& data);
		float get_angular_velocity_attractors(ProximitySector& data);
		float get_linear_velocity(ProximitySector& data);
		float normalize_angular_velocity(float w, float min, float max);
		bool  is_projection_inside(float distance, float angle, float width);

		// Callbacks
		void on_received_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_received_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_received_footprint(const geometry_msgs::PolygonStamped& data);
		void reconfigure_callback(cnbiros_shared_navigation::SharedDynamicsConfig &config, uint32_t level);

		// Utilities
		float	rad2deg(float radians);
		void	init_update_rate(float rate);
		bool	update_if_different(const float& first, float& second, float epsilon = 0.00001f);

	private:
		// Node handles
		ros::NodeHandle nh_;
		ros::NodeHandle	p_nh_;

		// Topics
		std::string t_repellors_;
		std::string t_attractors_;
		std::string	t_footprint_;
		std::string	t_velocity_;
		std::string	t_visual_velocity_;

		// Subscribers
		ros::Subscriber s_repellors_;
		ros::Subscriber s_attractors_;
		ros::Subscriber s_footprint_;

		// Publishers
		ros::Publisher	p_velocity_;
		ros::Publisher	p_visual_velocity_;

		// Proximity sectors
		ProximitySector	pr_repellors_;
		ProximitySector pr_attractors_;

		// Dynamic reconfigure
		DynamicReconfig					n_dynreconfig_server_;
		DynamicReconfig::CallbackType	n_dynreconfig_function_;

		// Robot
		std::string		robot_base_frame_;
		float			robot_lenght_;
		float			robot_width_;

		// General boolean states
		bool	is_data_available_;
		
		// General node variables
		bool		n_enable_repellors_;
		bool		n_enable_attractors_;
		float		n_update_rate_;
		ros::Rate* 	n_rate_;

		// Velocity Dynamics
		float	dyn_angular_velocity_min_;
		float	dyn_angular_velocity_max_;
		float	dyn_angular_repellors_strength_;
		float	dyn_angular_repellors_decay_;
		float	dyn_angular_repellors_occupancy_;
		float	dyn_linear_velocity_min_;
		float	dyn_linear_velocity_max_;
		float	dyn_linear_velocity_slope_;
		float	dyn_linear_safe_distance_;
		

		geometry_msgs::Twist		velocity_;
		geometry_msgs::PoseStamped		visual_velocity_;
};
		
	}
}

#endif
