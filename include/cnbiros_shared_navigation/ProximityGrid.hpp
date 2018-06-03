#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYGRID_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYGRID_HPP

// System includes
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace cnbiros {
    namespace navigation {

typedef std::vector<float>::iterator ProximityGridIt;
typedef std::vector<float>::const_iterator ProximityGridConstIt;

class ProximityGrid {

	public:
		
		ProximityGrid(float angle_min, float angle_max, float angle_inc,
					  float range_min, float range_max, std::string frame);
		ProximityGrid(void);

		ProximityGrid(const ProximityGrid&) = default;
		ProximityGrid& operator=(const ProximityGrid&) = default;
		ProximityGrid(ProximityGrid&&) = default;
		ProximityGrid& operator=(ProximityGrid&&) = default;

		ProximityGrid operator+(const ProximityGrid& g);
		
		virtual ~ProximityGrid(void);

		void SetAngleIncrement(float angle_inc);
		void SetAngleLimits(float angle_min, float angle_max);
		void SetRangeLimits(float range_min, float range_max);
		void SetFrame(std::string frame);

		int	  GetSize(void) const;
		float GetAngleMin(void) const;
		float GetAngleMax(void) const;
		float GetAngleIncrement(void) const;
		float GetRangeMin(void) const;
		float GetRangeMax(void) const;
		std::string GetFrame(void) const;

		
		bool SetGrid(const std::vector<float>& values);
		std::vector<float> GetGrid(void) const;

		bool SetSectorByPolar(const float distance, const float angle);
		bool SetSectorByCartesian(const float x, const float y);

		float GetSectorAngle(ProximityGridConstIt& it);
		float GetSectorValue(ProximityGridConstIt& it);
		float GetSectorAngle(unsigned int id);
		float GetSectorValue(unsigned int id);
		float GetSectorValue(float angle);
		int GetSectorId(float angle);
		

		bool  IsEmpty(void) const;
		void  Reset(void);

		std::string ToString(ProximityGridConstIt& it);
		std::string ToString(float angle);
		
	protected:
		void  init_proximity_grid(void);
		void  clear_proximity_grid(void);
		int	  get_number_sectors(float angle_min, float angle_max, float angle_inc);
		float get_angle_increment(float angle_min, float angle_max, int nsectors);
		bool  set_sector(float angle, float value);
		float get_sector_value(float angle);
		float get_sector_angle(unsigned int sector_id);
		int   get_sector_id(float angle);

	public:
		ProximityGridIt		Begin(void);
		ProximityGridIt		End(void);
		ProximityGridConstIt	Begin(void) const;
		ProximityGridConstIt	End(void) const;

	private:
		std::vector<float>		grid_;
	
		float					angle_max_;
		float					angle_min_;
		float					angle_inc_;
		std::string				frame_id_;

		float					range_min_;
		float					range_max_;
		
		int						nsectors_;


};

	}
}

#endif
