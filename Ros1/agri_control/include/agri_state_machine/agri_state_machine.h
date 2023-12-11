#ifndef AGRI_STATE_MACHINE_H
#define AGRI_STATE_MACHINE_H

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

class AgriStateMachine
{
	public:
		AgriStateMachine(std::string prefix="left");
		void spinner(void);
	private:
		void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr &ls);
		void gripperCallback(const std_msgs::Bool::ConstPtr &gs);
		bool checkGrasp();
		double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
		void moveApple();
		ros::NodeHandle nh_;
		ros::Subscriber gazebo_sub_;
		ros::Subscriber gripper_sub_;
		ros::Publisher gazebo_pub_;
		geometry_msgs::Pose robot_pose_;

		std::string prefix_;
		int machine_state_;
		int gripper_state_;
		uint apple_idx_;
		gazebo_msgs::LinkStates link_states_;
		
};

#endif /* AGRI_STATE_MACHINE_H */
