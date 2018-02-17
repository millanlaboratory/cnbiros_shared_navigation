#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_CPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_CPP

#include "cnbiros_shared_navigation/ProximitySector.hpp"

namespace cnbiros {
    namespace navigation {

ProximitySector::ProximitySector(int nsectors, float minangle, 
								 float maxangle, std::string frameid) {

	// Store sectors parameters
	this->SetResolution(nsectors);
	this->SetMinAngle(minangle);
	this->SetMaxAngle(maxangle);
	this->SetFrameId(frameid);

	// Initialize the inner vector
	this->init_sectors();
}

ProximitySector::ProximitySector(void) {
	ProximitySector(1, -M_PI/2.0f, M_PI/2.0f, "base_link");
};


ProximitySector::~ProximitySector(void) {}

void ProximitySector::SetResolution(int nsectors) {
	
	// Store sector parameters
	this->nsectors_ = nsectors;
	
	// Initialize the inner vector
	this->init_sectors();
}

void ProximitySector::SetMinAngle(float minangle) {

	// Store sector parameters
	this->min_angle_ = minangle;
	
	// Initialize the inner vector
	this->init_sectors();
}

void ProximitySector::SetMaxAngle(float maxangle) {

	// Store sector parameters
	this->max_angle_ = maxangle;
	
	// Initialize the inner vector
	this->init_sectors();
}

void ProximitySector::SetFrameId(std::string frameid) {
	
	// Store sector parameters
	this->frame_id_ = frameid;
}

std::string ProximitySector::GetFrameId(void) {
	return this->frame_id_;
}

float ProximitySector::GetMinAngle(void) {
	return this->min_angle_;
}

float ProximitySector::GetMaxAngle(void) {
	return this->max_angle_;
}

int ProximitySector::GetResolution(void) {
	return this->nsectors_;
}

void ProximitySector::Reset(void) {

	// Reset sectors
	this->reset_sectors();
}

bool ProximitySector::SetValues(const std::vector<float>& values) {
	this->sectors_ = values;
	return true;
}

std::vector<float> ProximitySector::GetValues(void) {
	return this->sectors_;
}

bool ProximitySector::SetByPolar(float angle, float radius) {

	// Set sectors
	return this->set_sectors(angle, radius);
}

bool ProximitySector::SetByCartesian(float x, float y) {
	
	float angle, radius;

	// Cartesian to Polar conversion
	angle  = atan2(x, y);
	radius = hypot(x, y); 

	// Set sectors
	return this->SetByPolar(angle, radius);
}

void ProximitySector::Dump(void) {

	// Dump sectors
	this->dump_sectors();
}

float ProximitySector::GetRadius(const ProximitySectorIt& it) {
	return (*it);
}

float ProximitySector::GetAngle(const ProximitySectorIt& it) {
	
	auto index = std::distance(this->sectors_.begin(), it);
	return this->min_angle_ + this->step_*(index + 0.5);
}

///******** Private methods to handle sector vector ********///
void ProximitySector::init_sectors(void) {

	// Compute the angular step between each sector
    this->step_ = (this->max_angle_ - this->min_angle_)/(float)this->nsectors_;

    // Initialize sector vector
    this->sectors_.reserve(this->nsectors_);
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

void ProximitySector::reset_sectors(void) {
    this->sectors_.clear();
    this->sectors_.assign(this->nsectors_, std::numeric_limits<float>::infinity());
}

bool ProximitySector::set_sectors(float angle, float radius) {

    unsigned int idsector;
    float cvalue;
	bool retval = false;
   
    // Determine the current sector, given the angle
    idsector = std::floor(angle/this->step_);

	if(idsector < this->sectors_.size()) {
		// Get the current value of the sector
    	cvalue = this->sectors_.at(idsector);

    	// Replace the value of the sector with the minimum between the current
    	// value and the given radius
    	this->sectors_.at(idsector) = std::min(cvalue, radius);

		retval = true;
	}

	return retval;
}

void ProximitySector::dump_sectors(void) {

    float sector_lower, sector_upper, sector_value;
	unsigned int i = 0;
	
	for(auto it=this->sectors_.begin(); it!=this->sectors_.end(); ++it) {
		sector_lower = (this->min_angle_ + i*this->step_)*180.0f/M_PI;
		sector_upper = sector_lower + this->step_*180.0f/M_PI;
		sector_value = (*it);
		ROS_INFO("ProximitySector %u [%2.1f<->%2.1f]: %f [m]", 
				  i, sector_lower, sector_upper, sector_value); 
		i++;
    }
}

	}
}

#endif

