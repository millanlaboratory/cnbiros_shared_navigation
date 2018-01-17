#include <ros/ros.h>
#include "cnbiros_wheelchair_navigation/CostMapToSectorGrid.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "costmap_to_sectorgrid");
	
	cnbiros::navigation::CostMapToSectorGrid t;
	ros::spin();
	
	return 0;
}
