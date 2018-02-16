#ifndef CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_COSTMAPTOPROXIMITYSECTOR_HPP

// System includes
#include <cmath>
#include <limits>

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <costmap_2d/cost_values.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"
#include "cnbiros_shared_navigation/CostMapSectorConfig.h"

namespace cnbiros {
    namespace navigation {

class CostMapToProximitySector {

    public:
		CostMapToProximitySector(void);
		virtual ~CostMapToProximitySector(void);
		
		bool configure(void);

	protected:
		void init_sectors(void);
		void reset_sectors(void);
		void set_sectors(float angle, float radius);
		void dump_sectors(void);
		void debug_dump_sectors(void);
    
	private:
		void on_received_costmap(const nav_msgs::OccupancyGrid& msgin);
		void on_dynamic_reconfiguration(cnbiros_shared_navigation::CostMapSectorConfig &config, 
										uint32_t level);

    protected:
		ros::NodeHandle			nh_;
  		ros::NodeHandle			private_nh_;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		std::string		stopic_;
		std::string 	ptopic_;

		tf::TransformListener	listener_;
		
		dynamic_reconfigure::Server<cnbiros_shared_navigation::CostMapSectorConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::CostMapSectorConfig>::CallbackType f_;

		std::string frame_id_;
		float threshold_;
		float min_angle_;
		float max_angle_;
		float step_;
		int nsectors_;
		
		std::vector<float> sectors_;

};



    }
}

#endif
