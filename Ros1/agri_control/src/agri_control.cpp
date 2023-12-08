#include "agri_control/agri_control.h"

AgriControl::AgriControl()
{
    joint_sub_ = nh_.subscribe("/joint_states", 1, &AgriControl::initJointCallback, this); 
    goal_sub_ = nh_.subscribe("/desired_robot_trajectory", 1, &AgriControl::trajectoryCallback, this); 
    
    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 1);

}

void AgriControl::initJointCallback(const sensor_msgs::JointState::ConstPtr &js)
{
    trajectory_goal_.joint_names = js->name;
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(js->name.size());

    std::vector<double> tmp_v = {M_PI*0.6, -M_PI*0.6, 0,  -M_PI*0.5, -M_PI*0.5, 0};
    for (uint i = 0; i < 6; i++)
    {
        p.positions[i] = tmp_v[i];
        p.positions[i+6] = tmp_v[i];
        joints_map_[js->name[i]] = js->position[i];
    } 
    p.positions.back() = 0;
    p.time_from_start = ros::Duration(1);
    trajectory_goal_.points.push_back(p);
    ros::Duration(1).sleep();
    trajectory_pub_.publish(trajectory_goal_);
    ros::Duration(1).sleep();
    joint_sub_ = nh_.subscribe("/joint_states", 1, &AgriControl::jointCallback, this); 
}

void AgriControl::jointCallback(const sensor_msgs::JointState::ConstPtr &js)
{
    actual_js_ = *js;
    for(uint i = 0; i < js->name.size(); i++)
        joints_map_[js->name[i]] = js->position[i];
}

void AgriControl::trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &traj)
{
    trajectory_goal_.points.resize(traj->points.size());
    for(uint i = 0; i < trajectory_goal_.points.size(); i++)
    {
        trajectory_goal_.points[i].time_from_start = traj->points[i].time_from_start; 

        static std::unordered_map<std::string, double>::iterator it;
        for (uint j = 0; j < traj->joint_names.size(); j++)
        {
            it = joints_map_.find(traj->joint_names[j]);
            if (it != joints_map_.end())
            {
               it->second = traj->points[i].positions[j];
            } 
        }
        for (uint j = 0; j < actual_js_.position.size(); j++)
            trajectory_goal_.points[i].positions[j] = joints_map_[actual_js_.name[j]];   
    }
    trajectory_pub_.publish(trajectory_goal_);

}

void AgriControl::spinner()
{
    ros::spinOnce();
}
