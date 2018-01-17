#ifndef CNBIROS_NAVIGATION_COSTMAP_TO_SECTORGRID_CPP
#define CNBIROS_NAVIGATION_COSTMAP_TO_SECTORGRID_CPP

#include "cnbiros_wheelchair_navigation/CostMapToSectorGrid.hpp"

namespace cnbiros {
    namespace navigation {

CostMapToSectorGrid::CostMapToSectorGrid(void) : private_nh_("~"), listener(ros::Duration(10)) {


    unsigned int nsectors;
    float sector_min_angle, sector_max_angle;

    this->obstacle_marker_ = 90.0f;
    this->polar_frame_id_ = "base_link";
    this->stopic_ = "/move_base/local_costmap/costmap";
    this->ptopic_ = "/polar";

    // Get sector parameters and initialize sector vector
    nsectors	     = 8;
    sector_min_angle = -M_PI/2.0f;
    sector_max_angle = M_PI/2.0f;
    this->init_sectors(nsectors, sector_min_angle, sector_max_angle);

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 50, 
				    &CostMapToSectorGrid::callback, this);
    this->pub_ = this->nh_.advertise<cnbiros_wheelchair_navigation::SectorGrid>(
				    this->ptopic_, 1000);
//    this->pub_ = this->nh_.advertise<cnbiros_wheelchair_navigation::PolarGrid>(
//				    this->ptopic_, 1000);
}

CostMapToSectorGrid::~CostMapToSectorGrid(void) {}

void CostMapToSectorGrid::init_sectors(unsigned int nsectors, float min_angle, float max_angle) {

    float sector_step;

    sector_step_ = (max_angle - min_angle)/(float)nsectors;

    // Initialize sector vector
    this->sectors_.reserve(nsectors);
    this->sectors_.assign(nsectors, std::numeric_limits<float>::infinity());

    // Store sector parameters
    this->nsectors_	    = nsectors;
    this->sector_min_angle_ = min_angle;
    this->sector_max_angle_ = max_angle;
    this->sector_step_	    = sector_step_;
}

void CostMapToSectorGrid::fill_sectors(float angle, float radius) {

    unsigned int idsector;
    float cvalue;
   
    // Determin the current sector, given the angle
    idsector = std::floor(angle/this->sector_step_);

    //printf("angle: %f\n", angle);
    //printf("step: %f\n", this->sector_step_);
    //printf("Id sector: %u\n", idsector);
    // Get the current value of the sector
    cvalue = this->sectors_.at(idsector);

    // Replace the value of the sector with the minimum between the current
    // value and the given radius
    this->sectors_.at(idsector) = std::min(cvalue, radius);
}

void CostMapToSectorGrid::reset_sectors(void) {
    this->sectors_.clear();
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void CostMapToSectorGrid::callback(const nav_msgs::OccupancyGrid& data_in) {

    grid_map::GridMap map;
    float value;
    float radius, angle;
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;
   
    ROS_INFO("New costmap");
    if(grid_map::GridMapRosConverter::fromOccupancyGrid(data_in, "costmap", map) == false) {
	ROS_ERROR("Cannot convert occupancy grid to grid map");
	return;
    }

    // Resetting current sectors
    this->reset_sectors();

    grid_map::Position position;
    grid_map::Matrix& data = map["costmap"];
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
	if(map.getPosition(index, position) == false) {
	    ROS_ERROR("Cannot get position in grid map");
	    continue;
	}
	value = map.at("costmap", index);
	if(value >= 90.0f) {

	    map_point.header.frame_id = data_in.header.frame_id;
	    map_point.point.x = position(0);
	    map_point.point.y = position(1);
	    map_point.point.z = 0.0f;
	    
	    try {
		this->listener.transformPoint(this->polar_frame_id_, map_point, base_point);
		
		if(base_point.point.x >= 0) {
		    angle  = atan2(base_point.point.x, -base_point.point.y);
		    radius = hypot(base_point.point.x, base_point.point.y); 

		    this->fill_sectors(angle, radius);
		}
	    } catch(tf::TransformException& ex) {
		ROS_ERROR("Cannot transform map to base point: %s", ex.what());
	    }
	}
    }

    unsigned int i = 0;
    float sector_lower, sector_upper, sector_value;
    for(auto it=this->sectors_.begin(); it!=this->sectors_.end(); ++it) {
	sector_lower = (this->sector_min_angle_ + i*this->sector_step_)*180.0f/M_PI;
	sector_upper = sector_lower + this->sector_step_*180.0f/M_PI;
	sector_value = (*it);
    
	ROS_INFO("Sector grid %u [%2.1f<->%2.1f]: %f [m]", i, sector_lower, 
			    sector_upper, sector_value); 
	i++;
    }


    cnbiros_wheelchair_navigation::SectorGrid data_out;

    data_out.header.frame_id = this->polar_frame_id_;
    data_out.header.stamp    = ros::Time::now();

    data_out.values	 = this->sectors_;
    data_out.min_angle	 = this->sector_min_angle_;
    data_out.max_angle	 = this->sector_max_angle_;
    data_out.nsectors	 = this->nsectors_;
    data_out.step	 = this->sector_step_;

    this->pub_.publish(data_out);
}

    }
}

#endif
