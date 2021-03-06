#ifndef CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_HPP
#define CNBIROS_SHAREDNAVIGATION_PROXIMITYSECTOR_HPP

// System includes
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace cnbiros {
    namespace navigation {

// Type traits for the iterators
typedef std::vector<float>::iterator ProximitySectorIt;
typedef std::vector<float>::const_iterator ProximitySectorConstIt;

/*!
 *	ProximitySector is a map in polar coordinates holding float values. The map
 *	is defined by sectors within a given angular range and with a given angular
 *	resolution.  For each sector, only the most proximal values are stored.
 *
 */
class ProximitySector {

	public:
		
		/*!
		 * Constructor.
		 * @param nsectors	Number of sectors defined the map resolution
		 * @param minangle	Minimum angle value defined in the map
		 * @param maxangle	Maximum angle value defined in the map
		 * @param frameid	Frame id of the map
		 */
		ProximitySector(int nsectors, float minangle, float maxangle, std::string frameid);

		/*!
		 * Empty constructor. 
		 * Default values:
		 *	- nsectors: 3
		 *	- minangle: 0.0 degrees
		 *	- maxangle: 180.0 degrees
		 *	- frameid:  "base_link"
		 */
		ProximitySector(void);

		/*!
		 * Default copy assign and copy constructors.
		 */
		ProximitySector(const ProximitySector&) = default;
		ProximitySector& operator=(const ProximitySector&) = default;
		ProximitySector(ProximitySector&&) = default;
		ProximitySector& operator=(ProximitySector&&) = default;
		
		/*!
		 * Destructor.
		 */
		virtual ~ProximitySector(void);

		/*!
		 * Set the frame id of the map.
		 * @param frameid	the name of the frame
		 */
		void SetFrameId(std::string frameid);

		/*!
		 * Set the angular resolution of the map.
		 * @param nsectors	Number of sectors. The angular resolution is
		 * computed accordingly.
		 */
		void SetResolution(int nsectors);

		/*!
		 * Set the minimum angle of the map with respect to the heading
		 * direction.
		 * @param min_angle	Minimum angle in the sector map
		 */
		void SetMinAngle(float min_angle);
		
		/*!
		 * Set the maximum angle of the map with respect to the heading
		 * direction.
		 * @param max_angle	Maximum angle in the sector map
		 */
		void SetMaxAngle(float max_angle);

		/*!
		 * Get the frame of the map.
		 * @return The frame name of the map
		 */
		std::string GetFrameId(void) const;

		/*!
		 * Get the minimum angle of the map.
		 * @return The minimum angle of the map
		 */
		float GetMinAngle(void) const;

		/*!
		 * Get the maximum angle of the map.
		 * @return The maximum angle of the map
		 */
		float GetMaxAngle(void) const;

		/*!
		 * Get the resolution of the map.
		 * @return The resolution of the map in number of sectors
		 */
		int	  GetResolution(void) const;

		/*!
		 * Get the sector step
		 * @return The angular step of the sectors
		 */
		float GetStep(void) const;
		
		/*!
		 * Set the values of the sector maps. The current values are cleared.
		 * Other maps characteristics are not set (e.g., the minimum and maximum
		 * angles).
		 * @return True if the values have been properly set.
		 */
		bool SetValues(const std::vector<float>& values);

		/*!
		 * Get the current values of the map as std::vector
		 * @return The current values of the map
		 */
		std::vector<float> GetValues(void) const;

		/*!
		 * Get the related angle of the sector identified by a given
		 * ProximitySector iterator.
		 * @param it Sector iterator
		 * @return The angle associated to the iterator position
		 */
		float GetAngle(ProximitySectorConstIt& it);

		/*!
		 * Get the related value of the sector identified by a given
		 * ProximitySector iterator.
		 * @param it Sector iterator
		 * @return The value of the sector associated to the iterator position
		 */
		float GetRadius(ProximitySectorConstIt& it) const;

		/*!
		 * Get the related sector identified by a given angle
		 * @param angle The given angle
		 * @return The id of the sector associated to the given angle. -1 if the
		 * angle is out of range
		 */
		int GetSector(float angle) const;
		
		/*!
		 * Get the value of the sector map at a given angle.
		 * @param angle	The given angle
		 * @return The value of the map at the requested angle
		 */
		float At(float angle) const;

		/*!
		 * Set a value in the sector map by polar coordinates (if within the
		 * sectors range).
		 * @param angle	Given angle
		 * @param radius Given radius (value to be set)
		 * @return True if the value has been set.
		 */
		bool  SetByPolar(float angle, float radius);
		
		/*!
		 * Set a value in the sector map by cartesian coordinates (if within the
		 * sectors range).
		 * @param x	Position along x-axis
		 * @param y Position along y-axis
		 * @return True if the value has been set.
		 */
		bool  SetByCartesian(float x, float y);

		/*!
		 * Return if the vector is empty
		 * @return True if succesful, false otherwise
		 */
		bool  IsEmpty(void) const;

		/*!
		 * Reset all the value in the sector map. The other map parameters are
		 * untouched.
		 */
		void  Reset(void);

		/*!
		 * Get value of a given sector in string format (for debugging)
		 * @param it	Sector iterator
		 * @return The value of the given sector ( Sector SectorId
		 * [SectorLower<->SectorUpper]: Distance [m] )
		 */
		std::string ToString(ProximitySectorConstIt& it);
		
		/*!
		 * Get value of a given sector in string format (for debugging)
		 * @param angle	Requested angle
		 * @return The value of the given sector ( Sector SectorId
		 * [SectorLower<->SectorUpper]: Distance [m] )
		 */
		std::string ToString(float angle);
		
	protected:
		/*!
		 * Initialize the sector data.
		 */
		void init_sectors(void);

		/*!
		 * Reset the sector data.
		 */
		void reset_sectors(void);

		/*!
		 * Set values in the sector data.
		 */
		bool set_sectors(float angle, float radius);

		/*!
		 * Get value in a given angle.
		 */
		void get_sectors(float angle);

	public:
		//! Iterator pointing to the begin of the sector map
		ProximitySectorIt		Begin(void);

		//! Iterator pointing to the end of the sector map
		ProximitySectorIt		End(void);
		
		//! Constant iterator pointing to the begin of the sector map
		ProximitySectorConstIt	Begin(void) const;
		
		//! Constant iterator pointing to the end of the sector map
		ProximitySectorConstIt	End(void) const;

	private:
		//! Frame id of the sector map
		std::string				frame_id_;
		
		//! Values of the sector map
		std::vector<float>		sectors_;
		
		//! Maximum angle of the sector map
		float					max_angle_;
		
		//! Minimum angle of the sector map
		float					min_angle_;
		
		//! Angular resolution of the sector map
		float					step_;
		
		//! Number of sectors in the sector map
		int						nsectors_;


};

	}
}

#endif
