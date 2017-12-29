#include <ros/ros.h>
#include "cnbiros_wheelchair_navigation/CostMapToPolarGrid.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "costmap_to_polargrid");
	
	cnbiros::navigation::CostMapToPolarGrid t;
	ros::spin();
	
	return 0;
}
