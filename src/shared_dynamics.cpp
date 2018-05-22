#include <ros/ros.h>
#include "cnbiros_shared_navigation/SharedDynamics.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "shared_actions");
	
	cnbiros::navigation::SharedDynamics shareddynamics;

	shareddynamics.Run();

	return 0;
}
