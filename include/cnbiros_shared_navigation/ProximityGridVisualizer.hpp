#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDVISUALIZER_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYGRIDVISUALIZER_HPP

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// Package includes
#include "cnbiros_shared_navigation/ProximityGrid.hpp"
#include "cnbiros_shared_navigation/ProximityGridMsg.h"
#include "cnbiros_shared_navigation/ProximityGridConverter.hpp"

namespace cnbiros {
    namespace navigation {

class ProximityGridVisualizer {

    public:
		ProximityGridVisualizer(void);
		virtual ~ProximityGridVisualizer(void);
		
		virtual bool configure(void);
    
	private:
		void on_received_grid(const cnbiros_shared_navigation::ProximityGridMsg& msg);

    protected:
		ros::NodeHandle	nh_;
  		ros::NodeHandle	private_nh_;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		std::string		stopic_;
		std::string 	ptopic_;

		float radius_;
};



    }
}

#endif
