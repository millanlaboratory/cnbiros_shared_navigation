#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYGRID_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYGRID_CPP

#include "cnbiros_shared_navigation/ProximityGrid.hpp"

namespace cnbiros {
    namespace navigation {

ProximityGrid::ProximityGrid(float angle_min, float angle_max, float angle_inc,
							 float range_min, float range_max, std::string frame) {

	// Store sectors parameters
	this->angle_min_ = angle_min;
	this->angle_max_ = angle_max;
	this->nsectors_  = this->get_number_sectors(this->angle_min_, this->angle_max_, angle_inc);
	this->angle_inc_ = this->get_angle_increment(this->angle_min_, this->angle_max_, this->nsectors_);
	this->range_min_ = range_min;
	this->range_max_ = range_max;
	this->frame_id_  = frame;

	// Initialize the inner vector
	this->init_proximity_grid();
}

ProximityGrid::ProximityGrid(void) {
	// Store sectors parameters
	this->angle_min_ = -M_PI/2.0f;
	this->angle_max_ =  M_PI/2.0f;
	this->nsectors_  = this->get_number_sectors(this->angle_min_, this->angle_max_, M_PI/9.0f);
	this->angle_inc_ = this->get_angle_increment(this->angle_min_, this->angle_max_, this->nsectors_);
	this->range_min_ =  0.0f;
	this->range_max_ =  6.0f;
	this->frame_id_  =  "base_link";

	// Initialize the inner vector
	this->init_proximity_grid();
};

ProximityGrid::~ProximityGrid(void) {}

ProximityGrid ProximityGrid::operator+(const ProximityGrid& g) {

	ProximityGrid			grid;
	ProximityGrid			other(g);
	ProximityGridConstIt	it;
	float					cangle,	cradius;

	grid.SetAngleLimits(this->GetAngleMin(), this->GetAngleMax());
	grid.SetAngleIncrement(this->GetAngleIncrement());
	grid.SetFrame(this->GetFrame());
	grid.SetRangeLimits(this->GetRangeMin(), this->GetRangeMax());
	grid.SetGrid(this->GetGrid());

	for(it = other.Begin(); it != other.End(); ++it) {

		cangle  = other.GetSectorAngle(it);
		cradius = other.GetSectorValue(it); 

		grid.SetSectorByPolar(cangle, cradius);
	}

	return grid;
}

void ProximityGrid::SetAngleIncrement(float angle_inc) {

	// Given the desired increment compute the number of sectors
	this->nsectors_ = this->get_number_sectors(this->angle_min_, this->angle_max_, angle_inc);

	// Re-compute the real angle increment given the above number of sectors
	this->angle_inc_ = this->get_angle_increment(this->angle_min_, this->angle_max_, this->nsectors_);

	// Re-Initialize the inner vector
	this->init_proximity_grid();
}

void ProximityGrid::SetAngleLimits(float angle_min, float angle_max) {

	// Store sector parameters
	this->angle_min_ = angle_min;
	this->angle_max_ = angle_max;
	
	// Re-compute the number of sectors
	this->nsectors_ = this->get_number_sectors(this->angle_min_, this->angle_max_, this->angle_inc_);

	// Re-compute the angle increment given the above number of sectors
	this->angle_inc_ = this->get_angle_increment(this->angle_min_, this->angle_max_, this->nsectors_);

	// Initialize the inner vector
	this->init_proximity_grid();
}

void ProximityGrid::SetRangeLimits(float range_min, float range_max) {

	// Store sector parameters
	this->range_min_ = range_min;
	this->range_max_ = range_max;
}

void ProximityGrid::SetFrame(std::string frame) {
	this->frame_id_ = frame;
}

float ProximityGrid::GetAngleMin(void) const {
	return this->angle_min_;
}

float ProximityGrid::GetAngleMax(void) const {
	return this->angle_max_;
}

float ProximityGrid::GetAngleIncrement(void) const {
	return this->angle_inc_;
}

float ProximityGrid::GetRangeMin(void) const {
	return this->range_min_;
}

float ProximityGrid::GetRangeMax(void) const {
	return this->range_max_;
}

int ProximityGrid::GetSize(void) const {
	return this->nsectors_;
}

std::string ProximityGrid::GetFrame(void) const {
	return this->frame_id_;
}


bool ProximityGrid::IsEmpty(void) const {
	return this->grid_.empty();
}

void ProximityGrid::Reset(void) {

	// Reset sectors
	this->clear_proximity_grid();
}

bool ProximityGrid::SetGrid(const std::vector<float>& values) {
	this->grid_ = values;
	return true;
}

std::vector<float> ProximityGrid::GetGrid(void) const {
	return this->grid_;
}

bool ProximityGrid::SetSectorByPolar(float angle, float value) {

	// Set sector
	return this->set_sector(angle, value);
}

bool ProximityGrid::SetSectorByCartesian(float x, float y) {
	
	float angle, radius;

	// Cartesian to Polar conversion
	angle  = std::atan2(x, y);
	radius = std::hypot(x, y); 

	// Set sectors
	return this->SetSectorByPolar(angle, radius);
}

std::string ProximityGrid::ToString(ProximityGridConstIt& it) {

	std::string message;
	float angle;

	angle = this->GetSectorAngle(it);
	message = this->ToString(angle);

	return message;
}

std::string ProximityGrid::ToString(float angle) {
	
	std::string message;
	int sector_id;
	float value;

	sector_id = this->GetSectorId(angle);
	value   = this->GetSectorValue(angle);

	message = "Sector " + std::to_string(sector_id) +  " [" +
			  std::to_string(angle + this->angle_inc_/2.0f) + "]: " +
			  std::to_string(value) + " [m]";

	return message;
}

float ProximityGrid::GetSectorValue(ProximityGridConstIt& it) {
	return (*it);
}

float ProximityGrid::GetSectorValue(unsigned int id) {
	return this->grid_.at(id);
}

float ProximityGrid::GetSectorAngle(ProximityGridConstIt& it) {

	unsigned int index = it - this->grid_.begin();
	return this->get_sector_angle(index);
}

int ProximityGrid::GetSectorId(float angle) {
	return this->get_sector_id(angle);
}

float ProximityGrid::GetSectorValue(float angle) {
	return this->get_sector_value(angle);
}

float ProximityGrid::GetSectorAngle(unsigned int id) {
	return this->get_sector_angle(id);
}

ProximityGridIt ProximityGrid::Begin(void) {
	return this->grid_.begin();
}

ProximityGridIt ProximityGrid::End(void) {
	return this->grid_.end();
}

ProximityGridConstIt ProximityGrid::Begin(void) const {
	return this->grid_.begin();
}

ProximityGridConstIt ProximityGrid::End(void) const {
	return this->grid_.end();
}

///******** Private/Protected methods to handle sector vector ********///
void ProximityGrid::init_proximity_grid(void) {

    // Initialize sector vector
    this->grid_.reserve(this->nsectors_);
    this->grid_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

int ProximityGrid::get_number_sectors(float angle_min, float angle_max, float angle_inc) {
	
	int nsectors;
	
	nsectors = std::round( (angle_max - angle_min)/angle_inc ); 

	// If even, make it odd
	if(nsectors %2 == 0)
		nsectors = nsectors + 1;

	return nsectors;
}

float ProximityGrid::get_angle_increment(float angle_min, float angle_max, int num_sectors) {

	return  (angle_max - angle_min)/(float)num_sectors;
}

void ProximityGrid::clear_proximity_grid(void) {
    this->grid_.clear();
    this->grid_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

int ProximityGrid::get_sector_id(float angle) {
	int	sector_id;

	if( (angle < this->angle_min_) || (angle > this->angle_max_)) {
		sector_id = -1;
	} else if( std::fabs(angle - this->angle_min_) < 0.0000001f) {
		sector_id = 0;
	} else if( std::fabs(angle - this->angle_max_) < 0.0000001f) {
		sector_id = this->nsectors_ - 1;
	} else {
		sector_id = std::floor((angle - this->angle_min_)/this->angle_inc_);
	}

	return sector_id;
}

bool ProximityGrid::set_sector(float angle, float value) {

    int sector_id;
    float cvalue;
	bool retval;
 
	// Check if the value is between range_min and range_max, otherwise
	// do not update the sector
	if(value < this->range_min_ || value > this->range_max_)
		return false;

	// Determine the current sector, given the angle
    sector_id = this->get_sector_id(angle);

	if(sector_id == -1) {
		retval = false;
	} else {
		// Get the current value of the sector
    	cvalue = this->grid_.at(sector_id);
		
		// Replace the value of the sector with the minimum between the current
    	// value and the given radius
    	this->grid_.at(sector_id) = std::min(cvalue, value);

		retval = true;
	}

	return retval;
}

float ProximityGrid::get_sector_value(float angle) {

	int sector_id;
	float value = std::nanf("");

	sector_id = this->get_sector_id(angle);

	if(sector_id != -1)
		value = this->grid_.at(sector_id);

	return value;

}

float ProximityGrid::get_sector_angle(unsigned int sector_id) {
	return (this->angle_min_ + this->angle_inc_*sector_id);
}

	}
}

#endif

