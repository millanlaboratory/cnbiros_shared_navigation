#ifndef CNBIROS_NAVIGATION_COSTMAP_TO_POLARGRID_CPP
#define CNBIROS_NAVIGATION_COSTMAP_TO_POLARGRID_CPP

#include "cnbiros_wheelchair_navigation/CostMapToPolarGrid.hpp"

namespace cnbiros {
    namespace navigation {

CostMapToPolarGrid::CostMapToPolarGrid(void) : private_nh_("~"), listener(ros::Duration(10)) {

    this->obstacle_marker_ = 90.0f;
    this->polar_frame_id_ = "base_link";
    this->stopic_ = "/move_base/local_costmap/costmap";
    this->ptopic_ = "/polar";

    this->sub_ = this->nh_.subscribe(this->stopic_, 50, &CostMapToPolarGrid::callback, this);
    this->pub_ = this->nh_.advertise<cnbiros_wheelchair_navigation::PolarGrid>(this->ptopic_, 1000);
}

CostMapToPolarGrid::~CostMapToPolarGrid(void) {}

void CostMapToPolarGrid::callback(const nav_msgs::OccupancyGrid& data_in) {

    grid_map::GridMap map;
    float value;
    float radius, angle;
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;
    std::vector<float> polar_radius;
    std::vector<float> polar_angle;
   
    ROS_INFO("New costmap");
    if(grid_map::GridMapRosConverter::fromOccupancyGrid(data_in, "costmap", map) == false) {
	ROS_ERROR("Cannot convert occupancy grid to grid map");
	return;
    }

    grid_map::Position position;
    grid_map::Matrix& data = map["costmap"];
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
	if(map.getPosition(index, position) == false) {
	    ROS_ERROR("Cannot get position in grid map");
	    continue;
	}
	value = map.at("costmap", index);
	if(value >= this->obstacle_marker_) {

	    map_point.header.frame_id = data_in.header.frame_id;
	    map_point.point.x = position(0);
	    map_point.point.y = position(1);
	    map_point.point.z = 0.0f;
	    
	    try {
		this->listener.transformPoint(this->polar_frame_id_, map_point, base_point);

		angle  = atan2(base_point.point.y, base_point.point.x);
		radius = hypot(base_point.point.x, base_point.point.y); 
		polar_radius.push_back(radius);
		polar_angle.push_back(angle);
		ROS_INFO("Obstacle at: (%f, %f) [m, deg]", radius, angle*180.0f/M_PI);
	    } catch(tf::TransformException& ex) {
		ROS_ERROR("Cannot transform map to base point: %s", ex.what());
	    }
	}

	
    }

    cnbiros_wheelchair_navigation::PolarGrid data_out;
    data_out.header.frame_id = this->polar_frame_id_;
    data_out.header.stamp = ros::Time::now();
    data_out.radius = polar_radius;
    data_out.angle  = polar_angle;
	
    this->pub_.publish(data_out);
}

    }
}

#endif
