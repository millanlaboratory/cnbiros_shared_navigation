#ifndef CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYGRID_HPP
#define CNBIROS_SHAREDNAVIGATION_LASERSCANTOPROXIMITYGRID_HPP

// Systema
#include <unordered_map>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>

// Package includes
#include "cnbiros_shared_navigation/ProximityGrid.hpp"
#include "cnbiros_shared_navigation/ProximityGridMsg.h"
#include "cnbiros_shared_navigation/ProximityGridConverter.hpp"
#include "cnbiros_shared_navigation/LaserScanGridConfig.h"

namespace cnbiros {
	namespace navigation {


class LaserScanToProximityGrid {

    public:
		LaserScanToProximityGrid(void);
		virtual ~LaserScanToProximityGrid(void);
		
		virtual bool configure(void);

		void Run(void);

	private:
		void on_received_laserscan(const sensor_msgs::LaserScanConstPtr& msgin, unsigned int id);
		void on_dynamic_reconfiguration(cnbiros_shared_navigation::LaserScanGridConfig &config, 
										uint32_t level);

		void init_update_rate(float rate);
		bool update_if_different(const float& first, float& second, float epsilon = 0.000001f);
		float rad2deg(float angle);
		float deg2rad(float angle);

    protected:
		ros::NodeHandle	nh_;
  		ros::NodeHandle	private_nh_;
		ros::Rate*		rate_;
		float			publish_frequency_;
		
		ros::Publisher					pub_;
		std::vector<ros::Subscriber>	sub_src_;
		
		std::string				 pub_topic_;
		std::vector<std::string> src_topic_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::LaserScanGridConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::LaserScanGridConfig>::CallbackType f_;
		
		ProximityGrid grid_;
		std::unordered_map<unsigned int, ProximityGrid>	grid_src_;

		tf::TransformListener listener_;
};

	}
}



#endif
