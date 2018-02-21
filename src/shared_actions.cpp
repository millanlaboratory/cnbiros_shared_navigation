#include <ros/ros.h>
#include "cnbiros_shared_navigation/SharedActions.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "shared_actions");
	
	cnbiros::navigation::SharedActions sharedactions;

	// Wait for Server
	sharedactions.WaitForServer();	

	sharedactions.Run();

	return 0;
}
