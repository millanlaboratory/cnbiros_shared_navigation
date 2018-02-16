#include <ros/ros.h>
#include "cnbiros_shared_navigation/CostMapToProximitySector.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "costmap_to_proximitysector");
	
	cnbiros::navigation::CostMapToProximitySector t;
	ros::spin();
	
	return 0;
}
