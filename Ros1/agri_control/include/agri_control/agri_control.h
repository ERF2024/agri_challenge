#ifndef AGRI_CONTROL_H
#define AGRI_CONTROL_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveGroupActionResult.h>

#include <unordered_map>

class AgriControl
{
	public:
		AgriControl();
		void spinner(void);
	private:
		void initJointCallback(const sensor_msgs::JointState::ConstPtr &js);
		void jointCallback(const sensor_msgs::JointState::ConstPtr &js);
		void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &traj);
		void moveitCallback(const moveit_msgs::MoveGroupActionResult::ConstPtr &res);
		void sendTrajectory(const trajectory_msgs::JointTrajectory &traj);
		ros::NodeHandle nh_;
		ros::Subscriber joint_sub_;
		ros::Subscriber goal_sub_;
		ros::Subscriber moveit_sub_;
		ros::Publisher trajectory_pub_;

		trajectory_msgs::JointTrajectory trajectory_goal_;
		sensor_msgs::JointState actual_js_;

    	std::unordered_map<std::string, double> joints_map_;

		// ros::Publisher pub;
		// std_msgs::Int32 msg;
};

#endif /* AGRI_CONTROL_H */
