#include <ros/ros.h>
#include "cnbiros_wheelchair_navigation/DynamicGoals.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "dynamic_goals");
	
	cnbiros::navigation::DynamicGoals dgoals;

	// Wait for Server
	dgoals.WaitForServer();	

	// Send start goal (forward)
	dgoals.Start();

	dgoals.Run();
	//ros::spin();

	return 0;
}
