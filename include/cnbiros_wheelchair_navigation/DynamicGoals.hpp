#ifndef CNBIROS_NAVIGATION_DYNAMICGOALS_HPP
#define CNBIROS_NAVIGATION_DYNAMICGOALS_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include "cnbiros_wheelchair_navigation/SectorGrid.h"
//#include "cnbiros_wheelchair_navigation/ObstacleStrength.h"
//#include "cnbiros_wheelchair_navigation/ObstacleDecay.h"
#include "cnbiros_wheelchair_navigation/DynamicGoalsParameters.h"
#include <dynamic_reconfigure/server.h>
#include "cnbiros_wheelchair_navigation/DynamicGoalsConfig.h"

#define EPSILON	0.00001

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class DynamicGoals {
    
    public:
		DynamicGoals(void);
		virtual ~DynamicGoals(void);

		void WaitForServer(void);

		void MakeGoal(void);
		void MakeGoal(float radius, float angle);
		void SendGoal(void);
		void CancelServerGoal(void);

		void Start(void);
		void Run(void);
		bool configure(void);

    protected:
		void reconfigure_callback(cnbiros_wheelchair_navigation::DynamicGoalsConfig &config, uint32_t level);

		virtual void callback(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual void user_callback(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_orientation(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_orientation_user(const cnbiros_wheelchair_navigation::SectorGrid& data);
		virtual float compute_position_exponential(const cnbiros_wheelchair_navigation::SectorGrid& data, float w);

		virtual float compute_position_logistic(const cnbiros_wheelchair_navigation::SectorGrid& data, float w);


		bool on_set_parameters(cnbiros_wheelchair_navigation::DynamicGoalsParameters::Request& req,
							   cnbiros_wheelchair_navigation::DynamicGoalsParameters::Response& res);

		void on_reset_command_user(const ros::TimerEvent& event);

		unsigned int angle2sector(float angle, float min_angle, float max_angle,
								  float step_angle, unsigned int nsectors);

	private:
		void init_command_timer(float timeout);
		void init_update_rate(float rate);
		void change_update_rate(float rate);

    protected:
		ros::NodeHandle nh_;
		ros::NodeHandle	private_nh_;

		std::string		obstacle_topic_;
		ros::Subscriber	subobstacles_;
		
		std::string		target_topic_;
		ros::Subscriber	subtargets_;

		ros::ServiceServer  srv_set_parameters_;
		ros::ServiceClient  srv_clear_costmap_;
		
		std::string		actionsrv_;
		MoveBaseClient*	actioncln_;

		float			obstacle_strength_;
		float			obstacle_decay_;
		float			obstacle_occupancy_;

		std::string		frame_id_;
		move_base_msgs::MoveBaseGoal	goal_;

		cnbiros_wheelchair_navigation::SectorGrid sector_data_;
		cnbiros_wheelchair_navigation::SectorGrid user_data_;


		float		command_timeout_;


		float max_goal_distance_;
		float slope_distance_;

		ros::Timer	command_timer_;
		

		dynamic_reconfigure::Server<cnbiros_wheelchair_navigation::DynamicGoalsConfig> cfgserver;
		dynamic_reconfigure::Server<cnbiros_wheelchair_navigation::DynamicGoalsConfig>::CallbackType f;

		float update_rate_;
		ros::Rate* rate_;
};

    }
}

#endif
