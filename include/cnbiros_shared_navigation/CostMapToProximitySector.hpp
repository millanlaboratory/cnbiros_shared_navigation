#ifndef CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
#include "cnbiros_shared_navigation/CostMapSectorConfig.h"

namespace cnbiros {
    namespace navigation {

class CostMapToProximitySector {

    public:
		CostMapToProximitySector(void);
		virtual ~CostMapToProximitySector(void);
		
		virtual bool configure(void);

	private:
		void on_received_costmap(const nav_msgs::OccupancyGrid& msgin);
		void on_dynamic_reconfiguration(cnbiros_shared_navigation::CostMapSectorConfig &config, 
										uint32_t level);

    protected:
		ros::NodeHandle	nh_;
  		ros::NodeHandle	private_nh_;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		std::string		stopic_;
		std::string 	ptopic_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::CostMapSectorConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::CostMapSectorConfig>::CallbackType f_;
		
		ProximitySector sector_;
		float			threshold_;
};



    }
}

#endif
