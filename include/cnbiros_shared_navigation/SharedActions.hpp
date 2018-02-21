#ifndef CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP
#define CNBIROS_SHAREDNAVIGATION_SHAREDACTIONS_HPP

// System includes
#include <exception>

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "cnbiros_shared_navigation/ProximitySector.hpp"
#include "cnbiros_shared_navigation/ProximitySectorMsg.h"
#include "cnbiros_shared_navigation/ProximitySectorConverter.hpp"
#include "cnbiros_shared_navigation/SharedActionsConfig.h"

#define SHAREDACTIONS_LOGISTIC_MINDISTANCE		0.01f

namespace cnbiros {
    namespace navigation {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*!
 * SharedActions implements a semi-autonomous navigation modality by creating
 * goals for the ROS navigation package (e.g., move_base) according to the a
 * given occupancy grid and possible commands of the user. Every time a new
 * occupancy grid or a user's command is available (as ProximitySector map),
 * they are stored. At each iteration, the SharedActions computes a new action
 * goal for the ROS navigation package based on a dynamic field algorithm and
 * taking into account the repellors in the current stored occupancy grid (e.g.,
 * obstacles) and the attractors in the current stored user's command. 
 * It checks the status of the current goal, and in the case of LOST or ABORTED
 * response, it ask for clearing the costmap.
 *
 * It works in parallel to a ROS navigation package (e.g., move_base).
 * Internally, it exploits the library object ProximitySector.
 *
 * It possible to dynamically reconfigure the parameters.
 *
 * Debug logging level prints out useful information for debugging.
 */
class SharedActions {
    
    public:
		/*! 
		 * Constructor.
		 */
		SharedActions(void);
		
		/*! 
		 * Destructor.
		 */
		virtual ~SharedActions(void);

		/*!
		 * Blocking method that waits for the action server to come up.
		 */
		virtual void WaitForServer(void);

		/*!
		 * Method that takes the stored occupancy grid and user's command and
		 * create a new goal accordingly.
		 */
		virtual void MakeGoal(void);

		/*!
		 * Method to send the goal to the action server.
		 */
		virtual void SendGoal(void);

		/*!
		 * Ask to cancel the current goals in the action server.
		 */
		virtual void CancelGoal(void);

		/*!
		 * Configure the class parameters.
		 */
		virtual bool configure(void);

		/*!
		 * Main loop of the class. At each iteration, a new goal is computed
		 * accordingly to the stored occupancy grid and user's command.
		 */
		virtual void Run(void);
		
		/*!
		 * Check if the object is running.
		 * @return True if success, false otherwise.
		 */
		bool IsRunning(void);

		/*!
		 * Method to set 'running' the state of the object.
		 */
		void Start(void);
		
		/*!
		 * Method to set 'stop' the state of the object.
		 */
		void Stop(void);

    protected:

		/*!
		 * Protected method to compute the orientation given the occupancy grid,
		 * according to Dynamic Field algorithm.
		 * @param data Occupancy grid in ProximitySector format
		 * @return Computed orientation
		 */
		float goal_orientation_repellors(ProximitySector& data);
		
		/*!
		 * Protected method to compute the orientation given the user's command,
		 * according to Dynamic Field algorithm.
		 * @param data User's command in ProximitySector format
		 * @return Computed orientation
		 */
		float goal_orientation_attractors(ProximitySector& data);
		
		/*!
		 * Protected method to limit an angle in a given range
		 * @param angle Input angle
		 * @param minangle Lower limit value
		 * @param maxangle Upper limit value
		 * @return Limited angle
		 */
		float goal_orientation_limits(float angle, float minangle, float maxangle);	

		/*!
		 * Protected method to compute the distance of the goal, given the
		 * occupancy grid and the current target orientation.
		 * It check the distance of the obstacle for the target orientation and
		 * then it applies a logistic function to determine the current distance
		 * of the goal.
		 * @param data Occupancy grid in ProximitySector format
		 * @param w Target goal orientation
		 * @return Computed distance of the goal
		 */
		float goal_position_logistic(ProximitySector& data, float w);
	
		/*!
		 * Protected method to dynamic reconfiguration.
		 */
		virtual void reconfigure_callback(cnbiros_shared_navigation::SharedActionsConfig &config, 
										  uint32_t level);

	private:
		void init_command_timer(float timeout);
		void init_update_rate(float rate);
		void change_update_rate(float rate);
		
		void on_receive_repellors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_receive_attractors(const cnbiros_shared_navigation::ProximitySectorMsg& data);
		void on_reset_command_user(const ros::TimerEvent& event);
		
		bool on_request_state_toggle(std_srvs::Trigger::Request& req,
									 std_srvs::Trigger::Response& res);

    protected:
		ros::NodeHandle nh_;
		ros::NodeHandle	private_nh_;

		std::string		reptopic_;
		std::string		atttopic_;
		ros::Subscriber	subrep_;
		ros::Subscriber	subatt_;
		
		std::string		actionsrv_;
		MoveBaseClient*	actioncln_;

		ros::ServiceClient  srv_clearcostmap_;
		ros::ServiceServer	srv_state_toggle_;
		
		move_base_msgs::MoveBaseGoal	goal_;

		bool		is_running_;
		bool		autostart_;

		std::string	frame_id_;
		float		robot_radius_;
		float		repellors_strength_;
		float		repellors_decay_;
		float		goal_max_distance_;
		float		goal_half_position_;
		float		command_timeout_;
		bool		is_data_available_;
		
		ProximitySector repellors_data_;
		ProximitySector attractors_data_;

		ros::Timer	command_timer_;

		dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig> cfgserver_;
		dynamic_reconfigure::Server<cnbiros_shared_navigation::SharedActionsConfig>::CallbackType f_;

		float update_rate_;
		ros::Rate* rate_;
};

    }
}

#endif
