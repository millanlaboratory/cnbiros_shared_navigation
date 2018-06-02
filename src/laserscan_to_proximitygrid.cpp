#include <ros/ros.h>
#include "cnbiros_shared_navigation/LaserScanToProximityGrid.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "laserscan_to_proximitygrid");
	
	cnbiros::navigation::LaserScanToProximityGrid t;
	
	t.Run();
	
	return 0;
}
