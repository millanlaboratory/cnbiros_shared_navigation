#ifndef CNBIROS_NAVIGATION_COSTMAP_TO_SECTORGRID_CPP
#define CNBIROS_NAVIGATION_COSTMAP_TO_SECTORGRID_CPP

#include "cnbiros_shared_navigation/CostMapToSectorGrid.hpp"

namespace cnbiros {
    namespace navigation {

CostMapToSectorGrid::CostMapToSectorGrid(void) : private_nh_("~"), listener(ros::Duration(10)) {


    unsigned int nsectors;
    float sector_min_angle, sector_max_angle;

	// Configure node
	this->configure();

    // Get sector parameters and initialize sector vector
    this->configure_sectors(this->nsectors_, this->min_angle_, this->max_angle_);

	// Initialize services
	this->srv_sector_number_    = this->private_nh_.advertiseService("set_num_sectors", 
												  &CostMapToSectorGrid::on_set_sector_number, this);
	this->srv_sector_minangle_  = this->private_nh_.advertiseService("set_min_angle", 
												  &CostMapToSectorGrid::on_set_sector_minangle, this);
	this->srv_sector_maxangle_  = this->private_nh_.advertiseService("set_max_angle", 
												  &CostMapToSectorGrid::on_set_sector_maxangle, this);
	this->srv_sector_threshold_ = this->private_nh_.advertiseService("set_threshold", 
												  &CostMapToSectorGrid::on_set_sector_threshold, this);

    // Initialize subscriber and publisher
    this->sub_ = this->nh_.subscribe(this->stopic_, 50, 
				    &CostMapToSectorGrid::callback, this);
    this->pub_ = this->nh_.advertise<cnbiros_shared_navigation::SectorGrid>(
				    this->ptopic_, 1000);
}

CostMapToSectorGrid::~CostMapToSectorGrid(void) {}

bool CostMapToSectorGrid::configure(void) {

	int nsectors;

	this->threshold_ = 100.0f;
	this->frame_id_  = "base_link";
	this->stopic_	 = "/move_base/local_costmap/costmap";
	this->ptopic_	 = "/sectorgrid";
	this->nsectors_	 = 40;
	this->min_angle_ = -M_PI/2.0f;
	this->max_angle_ = M_PI/2.0f;

	// Getting parameters
	this->private_nh_.getParam("threshold",  this->threshold_);
	this->private_nh_.getParam("frame_id",   this->frame_id_);
	this->private_nh_.getParam("costmap",    this->stopic_);
	this->private_nh_.getParam("sectorgrid", this->ptopic_);
	this->private_nh_.getParam("min_angle",  this->min_angle_);
	this->private_nh_.getParam("max_angle",  this->max_angle_);

	if(this->private_nh_.getParam("nsectors",   nsectors) == true)
		this->nsectors_ = (unsigned int)nsectors;

	ROS_INFO("SectorGrid frame_id:		    %s", this->frame_id_.c_str());
	ROS_INFO("SectorGrid threshold:			%f", this->threshold_);
	ROS_INFO("SectorGrid number of sectors:	%u", this->nsectors_);
	ROS_INFO("SectorGrid minimum angle:		%f", this->min_angle_);
	ROS_INFO("SectorGrid maximum angle:		%f", this->max_angle_);
	ROS_INFO("SectorGrid subscribed topic:  %s", this->stopic_.c_str());
	ROS_INFO("SectorGrid advertised topic:  %s", this->ptopic_.c_str());

	return true;
}

void CostMapToSectorGrid::configure_sectors(unsigned int nsectors, float min_angle, float max_angle) {

    this->step_ = (max_angle - min_angle)/(float)nsectors;

    // Initialize sector vector
    this->sectors_.reserve(nsectors);
    this->sectors_.assign(nsectors, std::numeric_limits<float>::infinity());
}

void CostMapToSectorGrid::update_sectors(float angle, float radius) {

    unsigned int idsector;
    float cvalue;
   
    // Determin the current sector, given the angle
    idsector = std::floor(angle/this->step_);

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
    float value, radius, angle;
    geometry_msgs::PointStamped map_point;
    geometry_msgs::PointStamped base_point;
    
	unsigned int i;
    float sector_lower, sector_upper, sector_value;
   
    //ROS_INFO("New costmap");
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
		if(value >= this->threshold_) {

		    map_point.header.frame_id = data_in.header.frame_id;
		    map_point.point.x = position(0);
		    map_point.point.y = position(1);
		    map_point.point.z = 0.0f;
		    
		    try {
				this->listener.transformPoint(this->frame_id_, map_point, base_point);
				
				if(base_point.point.x >= 0) {
				    angle  = atan2(base_point.point.x, -base_point.point.y);
				    radius = hypot(base_point.point.x, base_point.point.y); 

				    this->update_sectors(angle, radius);
				}
		    } catch(tf::TransformException& ex) {
				ROS_ERROR("Cannot transform map to base point: %s", ex.what());
		    }
		}
    }

	i = 0;
    for(auto it=this->sectors_.begin(); it!=this->sectors_.end(); ++it) {
		sector_lower = (this->min_angle_ + i*this->step_)*180.0f/M_PI;
		sector_upper = sector_lower + this->step_*180.0f/M_PI;
		sector_value = (*it);
    	
		//ROS_INFO("Sector grid %u [%2.1f<->%2.1f]: %f [m]", i, sector_lower, sector_upper, sector_value); 
		i++;
    }


    cnbiros_shared_navigation::SectorGrid data_out;

    data_out.header.frame_id = this->frame_id_;
    data_out.header.stamp    = ros::Time::now();

    data_out.values		= this->sectors_;
    data_out.min_angle	= this->min_angle_;
    data_out.max_angle	= this->max_angle_;
    data_out.nsectors	= this->nsectors_;
    data_out.step		= this->step_;

    this->pub_.publish(data_out);
}


bool CostMapToSectorGrid::on_set_sector_number(cnbiros_shared_navigation::SectorNumber::Request &req,
											   cnbiros_shared_navigation::SectorNumber::Response &res) {

	if(req.num_sectors <= 0) {
		ROS_ERROR("Number of sectors must be > 0");
		res.result = false;
	} else {
		this->nsectors_ = (unsigned int)req.num_sectors;
		this->configure_sectors(this->nsectors_, this->min_angle_, this->max_angle_);
		ROS_INFO("Updated the number of sectors to: %u", this->nsectors_);
		res.result = true;
	}

	return res.result;
}

bool CostMapToSectorGrid::on_set_sector_minangle(cnbiros_shared_navigation::SectorMinAngle::Request &req,
											     cnbiros_shared_navigation::SectorMinAngle::Response &res) {

	this->min_angle_ = req.min_angle;
	this->configure_sectors(this->nsectors_, this->min_angle_, this->max_angle_);
	ROS_INFO("Updated the minimum angle to: %f", this->min_angle_);
	res.result = true;

	return res.result;
}

bool CostMapToSectorGrid::on_set_sector_maxangle(cnbiros_shared_navigation::SectorMaxAngle::Request &req,
											     cnbiros_shared_navigation::SectorMaxAngle::Response &res) {

	this->max_angle_ = req.max_angle;
	this->configure_sectors(this->nsectors_, this->min_angle_, this->max_angle_);
	ROS_INFO("Updated the maximum angle to: %f", this->max_angle_);
	res.result = true;

	return res.result;
}

bool CostMapToSectorGrid::on_set_sector_threshold(cnbiros_shared_navigation::SectorThreshold::Request &req,
											      cnbiros_shared_navigation::SectorThreshold::Response &res) {

	this->threshold_ = req.threshold;
	ROS_INFO("Updated the threshold to: %f", this->threshold_);
	res.result = true;

	return res.result;
}
    }
}

#endif
