#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_HPP

// System includes
#include <cmath>
#include <limits>

// ROS includes
#include <ros/ros.h>


namespace cnbiros {
    namespace navigation {

typedef std::vector<float>::iterator ProximitySectorIt;
typedef std::vector<float>::const_iterator ProximitySectorConstIt;

class ProximitySector {

	public:
		// Constructor
		ProximitySector(int nsectors, float minangle, float maxangle, std::string frameid);

		// Empty constructor 
		// (default values: 3, -M_PI/2.0f, M_PI/2.0f, "base_link")
		ProximitySector(void);

		// Destructor
		virtual ~ProximitySector(void);

		// Default copy assign and constructors
		ProximitySector(const ProximitySector&) = default;
		ProximitySector& operator=(const ProximitySector&) = default;
		ProximitySector(ProximitySector&&) = default;
		ProximitySector& operator=(ProximitySector&&) = default;

		void SetFrameId(std::string frameid);
		void SetResolution(int nsectors);
		void SetMinAngle(float min_angle);
		void SetMaxAngle(float max_angle);

		std::string GetFrameId(void);
		float GetMinAngle(void);
		float GetMaxAngle(void);
		int	  GetResolution(void);

		bool SetValues(const std::vector<float>& values);
		std::vector<float> GetValues(void);

		float GetAngle(const  ProximitySectorIt& it);
		float GetRadius(const ProximitySectorIt& it);
		bool  SetByPolar(float angle, float radius);
		bool  SetByCartesian(float x, float y);

		void  Reset(void);
		void  Dump(void);
		
	protected:
		void init_sectors(void);
		void reset_sectors(void);
		bool set_sectors(float angle, float radius);
		void get_sectors(float angle);

		void dump_sectors(void);

	public:
		ProximitySectorIt		Begin(void);
		ProximitySectorIt		End(void);
		ProximitySectorConstIt	Begin(void) const;
		ProximitySectorConstIt	End(void) const;

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
