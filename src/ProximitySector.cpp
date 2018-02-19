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

std::string ProximitySector::GetFrameId(void) const {
	return this->frame_id_;
}

float ProximitySector::GetMinAngle(void) const {
	return this->min_angle_;
}

float ProximitySector::GetMaxAngle(void) const {
	return this->max_angle_;
}

int ProximitySector::GetResolution(void) const {
	return this->nsectors_;
}

float ProximitySector::GetStep(void) const {
	return this->step_;
}

bool ProximitySector::IsEmpty(void) const {
	return this->sectors_.empty();
}

void ProximitySector::Reset(void) {

	// Reset sectors
	this->reset_sectors();
}

bool ProximitySector::SetValues(const std::vector<float>& values) {
	this->SetResolution(values.size());
	this->sectors_ = values;
	return true;
}

std::vector<float> ProximitySector::GetValues(void) const {
	return this->sectors_;
}

bool ProximitySector::SetByPolar(float angle, float radius) {

	// Set sectors
	return this->set_sectors(angle, radius);
}

bool ProximitySector::SetByCartesian(float x, float y) {
	
	float angle, radius;

	// Cartesian to Polar conversion
	angle  = std::atan2(x, y) - M_PI/2.0f;
	radius = std::hypot(x, y); 

	// Set sectors
	return this->SetByPolar(angle, radius);
}

void ProximitySector::Dump(void) {

	// Dump sectors
	this->dump_sectors();
}

float ProximitySector::GetRadius(ProximitySectorConstIt& it) const {
	return (*it);
}

float ProximitySector::GetAngle(ProximitySectorConstIt& it) {

	unsigned int index = it - this->sectors_.begin();
	return (this->min_angle_ + this->step_*(index + 0.5));
}

int ProximitySector::GetSector(float angle) const {

	int	idsector;

	if( (angle < this->min_angle_) || (angle > this->max_angle_)) {
		idsector = -1;
	} else if(angle == this->min_angle_) {
		idsector = 0;
	} else if(angle == this->max_angle_) {
		idsector = this->nsectors_ - 1;
	} else {
		idsector = std::floor((angle - this->min_angle_)/this->step_);
	}

	return idsector;
}

float ProximitySector::At(float angle) const {

    int idsector;
	float cvalue = std::numeric_limits<float>::infinity();

    // Determine the current sector, given the angle
    //idsector = std::floor((angle-this->min_angle_)/this->step_);
	idsector = this->GetSector(angle);
	
	if(idsector < 0) {
		std::string error = "Requested angle (angle: " + std::to_string(angle*180.0f/M_PI)
							+ ") out of sector range";
		throw std::runtime_error(error);
	} else {
		cvalue = this->sectors_.at(idsector);
	}

	return cvalue;
}

ProximitySectorIt ProximitySector::Begin(void) {
	return this->sectors_.begin();
}

ProximitySectorIt ProximitySector::End(void) {
	return this->sectors_.end();
}

ProximitySectorConstIt ProximitySector::Begin(void) const {
	return this->sectors_.begin();
}

ProximitySectorConstIt ProximitySector::End(void) const {
	return this->sectors_.end();
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

    int idsector;
    float cvalue;
	bool retval;
  
	// Determine the current sector, given the angle
    idsector = this->GetSector(angle);
	
	if(idsector == -1) {
		retval = false;
	} else {
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

    float sector_lower, sector_upper;
	float cvalue, cangle;
	unsigned int csector;
	ProximitySectorConstIt it;
	
	for(it=this->Begin(); it!=this->End(); ++it) {
		cvalue  = this->GetRadius(it);
		cangle  = this->GetAngle(it);
		csector = this->GetSector(cangle);
		sector_lower = cangle - this->step_/2.0f;
		sector_upper = cangle + this->step_/2.0f;

		ROS_INFO("ProximitySector %u [%2.1f<->%2.1f]: %f [m]", 
				  csector, sector_lower*180.0f/M_PI,
				  sector_upper*180.0f/M_PI, cvalue); 
    }
}

	}
}

#endif

