#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_HPP

// System includes
#include <cmath>
#include <limits>

// ROS includes
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <costmap_2d/cost_values.h>

// Package include
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"

namespace cnbiros {
    namespace navigation {

typedef std::vector<float>::iterator ProximitySectorIt;
typedef std::vector<float>::const_iterator ProximitySectorConstIt;

class ProximitySector {

	public:
		ProximitySector(int nsectors, float minangle, float maxangle, std::string frameid);
		virtual ~ProximitySector(void);

		void SetFrameId(std::string frameid);
		void SetResolution(int nsectors);
		void SetRange(float min_angle, float max_angle);

		float GetAngle(const  ProximitySectorIt& it);
		float GetRadius(const ProximitySectorIt& it);
		void Set(float angle, float radius);
		void Reset(void);
		void Dump(void);

		bool FromOccupancyGrid(const nav_msgs::OccupancyGrid& msg, float threshold);
		bool FromPoint(const geometry_msgs::PointStamped& msg);
		bool FromMessage(const cnbiros_shared_navigation::ProximitySectorMsg& msg);

		cnbiros_shared_navigation::ProximitySectorMsg ToMessage(void);

	protected:
		void init_sectors(void);
		void reset_sectors(void);
		void set_sectors(float angle, float radius);
		void get_sectors(float angle);

		void dump_sectors(void);

	public:
		ProximitySectorIt		Begin(void);
		ProximitySectorIt		End(void);
		ProximitySectorConstIt	Begin(void) const;
		ProximitySectorConstIt	End(void) const;

	protected:
		tf::TransformListener	listener_;

	private:
		std::string				frame_id_;
		std::vector<float>		sectors_;
		float					max_angle_;
		float					min_angle_;
		float					step_;
		int						nsectors_;


};

	}
}

#endif
