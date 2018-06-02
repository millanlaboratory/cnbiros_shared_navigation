#include <ros/ros.h>
#include "cnbiros_shared_navigation/ProximityGridVisualizer.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "proximitysector_visualizer");
	
	cnbiros::navigation::ProximityGridVisualizer t;
	ros::spin();
	
	return 0;
}
