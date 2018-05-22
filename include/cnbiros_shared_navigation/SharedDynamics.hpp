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

		// Velocity Dynamics
		float get_angular_velocity_repellors(ProximitySector& data);
		float get_angular_velocity_attractors(ProximitySector& data);
		float get_linear_velocity(ProximitySector& data);
		float limit_angular_velocity(float w, float limit);

		// Callbacks
		void on_received_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_received_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void reconfigure_callback(cnbiros_shared_navigation::SharedDynamicsConfig &config, uint32_t level);

		// Utilities
		float rad2deg(float radians);

	private:
		// Node handles
		ros::NodeHandle nh_;
		ros::NodeHandle	p_nh_;

		// Topics
		std::string t_repellors_;
		std::string t_attractors_;

		// Subscribers
		ros::Subscriber s_repellors_;
		ros::Subscriber s_attractors_;

		// Proximity sectors
		ProximitySector	pr_repellors_;
		ProximitySector pr_attractors_;

		// Dynamic reconfigure
		DynamicReconfig					n_dynreconfig_server_;
		DynamicReconfig::CallbackType	n_dynreconfig_function_;

		// Robot
		std::string		robot_base_frame_;

		// General boolean states
		bool	is_data_available_;
		
		// General node variables
		bool	n_enable_repellors_;
		bool	n_enable_attractors_;
		float	n_update_rate_;

		// Velocity Dynamics
		float	dyn_angular_velocity_limit_;
};
		
	}
}

#endif
