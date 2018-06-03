#ifndef CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYGRID_HPP
#define CNBIROS_SHAREDNAVIGATION_POINTTOPROXIMITYGRID_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximityGrid.hpp"
#include "cnbiros_shared_navigation/ProximityGridMsg.h"
#include "cnbiros_shared_navigation/ProximityGridConverter.hpp"
#include "cnbiros_shared_navigation/PointGridConfig.h"

namespace cnbiros {
    namespace navigation {

class PointToProximityGrid {

    public:
		PointToProximityGrid(void);
		virtual ~PointToProximityGrid(void);
		
		virtual bool configure(void);

		void Run(void);
    
	private:
		void on_received_point(const geometry_msgs::PointStamped& msg);
		void on_dynamic_reconfiguration(cnbiros_shared_navigation::PointGridConfig &config, 
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
		
		ros::Publisher	pub_;
		std::vector<ros::Subscriber>	subs_;

		std::string 	ptopic_;
		std::vector<std::string> stopics_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::PointGridConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::PointGridConfig>::CallbackType f_;

		ProximityGrid				grid_;
		std::vector<ProximityGrid>	grid_sources_;

		tf::TransformListener listener_;
};



    }
}

#endif
