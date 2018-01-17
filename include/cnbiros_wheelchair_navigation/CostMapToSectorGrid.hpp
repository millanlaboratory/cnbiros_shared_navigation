#ifndef CNBIROS_NAVIGATION_COSTMAP_TO_SECTORGRID_HPP
#define CNBIROS_NAVIGATION_COSTMAP_TO_SECTORGRID_HPP

#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <costmap_2d/cost_values.h>
#include "cnbiros_wheelchair_navigation/SectorGrid.h"
#include "cnbiros_wheelchair_navigation/SectorNumber.h"
#include "cnbiros_wheelchair_navigation/SectorMaxAngle.h"
#include "cnbiros_wheelchair_navigation/SectorMinAngle.h"
#include "cnbiros_wheelchair_navigation/SectorThreshold.h"

namespace cnbiros {
    namespace navigation {

class CostMapToSectorGrid {

    public:
		CostMapToSectorGrid(void);
		virtual ~CostMapToSectorGrid(void);
		
		bool configure(void);

    private:
		void callback(const nav_msgs::OccupancyGrid& data_in);
		void configure_sectors(unsigned int nsectors, float min_angle, float max_angle);
		void update_sectors(float angle, float radius);
		void reset_sectors(void);

		bool on_set_sector_number(cnbiros_wheelchair_navigation::SectorNumber::Request &req,
								  cnbiros_wheelchair_navigation::SectorNumber::Response &res);
		bool on_set_sector_minangle(cnbiros_wheelchair_navigation::SectorMinAngle::Request &req,
								    cnbiros_wheelchair_navigation::SectorMinAngle::Response &res);
		bool on_set_sector_maxangle(cnbiros_wheelchair_navigation::SectorMaxAngle::Request &req,
									cnbiros_wheelchair_navigation::SectorMaxAngle::Response &res);
		bool on_set_sector_threshold(cnbiros_wheelchair_navigation::SectorThreshold::Request &req,
									 cnbiros_wheelchair_navigation::SectorThreshold::Response &res);

    protected:
		ros::NodeHandle nh_;
  		ros::NodeHandle private_nh_;
		tf::TransformListener listener;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		std::string		stopic_;
		std::string 	ptopic_;

		ros::ServiceServer srv_sector_number_;
		ros::ServiceServer srv_sector_minangle_;
		ros::ServiceServer srv_sector_maxangle_;
		ros::ServiceServer srv_sector_threshold_;

		float threshold_;
		std::string frame_id_;


		int nsectors_;
		float min_angle_;
		float max_angle_;
		float step_;
		std::vector<float> sectors_;


};



    }
}

#endif
