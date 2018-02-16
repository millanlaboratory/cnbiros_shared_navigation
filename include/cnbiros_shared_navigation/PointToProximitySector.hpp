#ifndef CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYSECTOR_HPP

// System includes
#include <cmath>
#include <limits>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/PointSectorConfig.h"

namespace cnbiros {
    namespace navigation {

class PointToProximitySector {

    public:
		PointToProximitySector(void);
		virtual ~PointToProximitySector(void);
		
		bool configure(void);
    
	private:
		void on_received_point(const geometry_msgs::PointStamped& msgin);
		void on_dynamic_reconfiguration(cnbiros_shared_navigation::PointSectorConfig &config, 
										uint32_t level);

    protected:
		ros::NodeHandle			nh_;
  		ros::NodeHandle			private_nh_;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		std::string		stopic_;
		std::string 	ptopic_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::PointSectorConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::PointSectorConfig>::CallbackType f_;

		ProximitySector*	sector_;
};



    }
}

#endif
