#include <ros/ros.h>
#include "cnbiros_shared_navigation/LaserScanToProximitySector.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "laserscan_to_proximitysector");
	
	cnbiros::navigation::LaserScanToProximitySector t;
	ros::spin();
	
	return 0;
}
