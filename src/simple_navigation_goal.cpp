#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <tf/tf.h>


#define CMD_LEFT   'a'
#define CMD_RIGHT  'd'
#define CMD_STOP  'q'

enum STATES {NB_ENABLE, NB_DISABLE};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


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
    float posx, posy;
    char key;
    int i =0;
    float angle;
    bool stop = false;
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }


    nonblock(NB_ENABLE);
    while(ros::ok()) {

	i = kbhit();
	if (i!=0) {
            key=fgetc(stdin);
        }
	switch(key) {
	    case CMD_LEFT:
		angle = -M_PI/4.0f;
		stop = false;
		break;
	    case CMD_RIGHT:
		angle = -3.0f*M_PI/4.0f;
		stop = false;
		break;
	    case CMD_STOP:
		stop = true;
		break;
	    default:
		angle = -M_PI/2.0f;
		stop = false;
	}
	if (stop == false) {
	    posx = -sin(angle);
	    posy = cos(angle);
	    ROS_INFO("Current goal: [%f, %f]\n", posx, posy);
	} else {
	    posx = 0.0f;
	    posy = 0.0f;
	    ROS_INFO("Stop\n");
	}

	move_base_msgs::MoveBaseGoal goal;

    	goal.target_pose.header.frame_id = "base_link";
    	goal.target_pose.pose.position.x = posx;
    	goal.target_pose.pose.position.y = posy;
    	//goal.target_pose.pose.orientation.w = 1.0f;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle-M_PI/2.0f);
	//ROS_INFO("Sending goals");

	goal.target_pose.header.stamp = ros::Time::now();
	ac.sendGoal(goal);

	//ros::Duration(1.0).sleep();
    }

    return 0;
}
