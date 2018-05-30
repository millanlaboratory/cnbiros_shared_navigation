#ifndef CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYSECTOR_HPP


// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
#include "cnbiros_shared_navigation/LaserScanSectorConfig.h"

namespace cnbiros {
	namespace navigation {


class LaserScanToProximitySector {

    public:
		LaserScanToProximitySector(void);
		virtual ~LaserScanToProximitySector(void);
		
		virtual bool configure(void);

	private:
		void on_received_laserscan(const sensor_msgs::LaserScan& msgin);
		void on_dynamic_reconfiguration(cnbiros_shared_navigation::LaserScanSectorConfig &config, 
										uint32_t level);

    protected:
		ros::NodeHandle	nh_;
  		ros::NodeHandle	private_nh_;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		ros::Publisher	vpub_;
		std::string		stopic_;
		std::string 	ptopic_;
		std::string		vtopic_;
		sensor_msgs::LaserScan	vsector_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::LaserScanSectorConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::LaserScanSectorConfig>::CallbackType f_;
		
		ProximitySector sector_;
};

	}
}



#endif
