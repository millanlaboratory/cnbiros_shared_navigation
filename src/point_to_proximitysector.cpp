#include <ros/ros.h>
#include "cnbiros_shared_navigation/PointToProximitySector.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "point_to_proximitysector");
	
	cnbiros::navigation::PointToProximitySector t;
	ros::spin();
	
	return 0;
}
