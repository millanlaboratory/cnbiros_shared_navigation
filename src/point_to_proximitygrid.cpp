#include <ros/ros.h>
#include "cnbiros_shared_navigation/PointToProximityGrid.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "point_to_proximitygrid");
	
	cnbiros::navigation::PointToProximityGrid t;

	t.Run();

	return 0;
}
