#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <limits>

#include <ros/ros.h>
#include "cnbiros_shared_navigation/SectorGrid.h"

#define CMD_FORWARD	'w'
#define CMD_LEFT	'a'
#define CMD_RIGHT	'd'
#define CMD_QUIT	'q'

enum STATES {NB_ENABLE, NB_DISABLE};


int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state) {
    struct termios ttystate;
 
    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);
 
    if (state==NB_ENABLE)
    {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state==NB_DISABLE)
    {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}


int main(int argc, char** argv){
    char key;
    int i =0;
    float angle;
    bool iscommand = false;
	bool isquit    = false;
   
	unsigned int idsector;

	ros::init(argc, argv, "user_goal");
	ros::NodeHandle node;
	ros::Publisher	pub;
	ros::Rate r(10);

	
	unsigned int	nsectors	= 9;
	float			min_angle	= -M_PI/2.0f;
	float			max_angle	=  M_PI/2.0f;
	std::string		frame_id	= "base_link";
	float			radius		= 1.0f;
	float			step        = (max_angle - min_angle)/(float)nsectors;
	std::string		topic		= "/user";


	// Initialize SectorGrid message
	cnbiros_shared_navigation::SectorGrid user_data;
	user_data.header.frame_id	= frame_id;
	user_data.min_angle			= min_angle;
	user_data.max_angle			= max_angle;
	user_data.nsectors			= nsectors;
	user_data.step				= step;

	user_data.values.reserve(nsectors);
	user_data.values.assign(nsectors, std::numeric_limits<float>::infinity());
	
	// Advertise topic
    pub = node.advertise<cnbiros_shared_navigation::SectorGrid>(topic, 1000);

    nonblock(NB_ENABLE);
    while(ros::ok() & isquit == false) {

		i = kbhit();
		if (i!=0) {
    	    key=fgetc(stdin);
    	}

		user_data.values.clear();
    	user_data.values.assign(nsectors, std::numeric_limits<float>::infinity());

		switch(key) {
		    case CMD_FORWARD:
				angle = 0.0f;
				iscommand = true;
				key = 'v';
				ROS_INFO("User command FORWARD at: %2.1f", (angle)*180.0f/M_PI);
				break;
		    case CMD_LEFT:
				angle = -M_PI/8.0f;
				iscommand = true;
				key = 'v';
				ROS_INFO("User command LEFT at: %2.1f", (angle)*180.0f/M_PI);
				break;
		    case CMD_RIGHT:
				angle = M_PI/8.0f;
				iscommand = true;
				key = 'v';
				ROS_INFO("User command RIGHT at: %2.1f", (angle)*180.0f/M_PI);
				break;
			case CMD_QUIT:
				isquit	  = true;
				iscommand = false;
		    default:
				iscommand = false;
		}


		if(iscommand == true) {
			idsector = std::trunc((angle + std::fabs(min_angle))/step);
			idsector = idsector >= nsectors ? nsectors - 1 : idsector;
			user_data.values.at(idsector) = radius;

			ROS_INFO("Command at %2.3f (idsector: %u)", angle*180.0f/M_PI, idsector);
		
			user_data.header.stamp = ros::Time::now();
			
			pub.publish(user_data);
			iscommand = false;
		}

		ros::spinOnce();
		r.sleep();
    }

    return 0;
}
