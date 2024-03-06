#include "agri_control/agri_control.h"

AgriControl::AgriControl(std::string prefix)
{
    prefix_ = prefix;
    joint_sub_ = nh_.subscribe("/joint_states", 1, &AgriControl::initJointCallback, this);
    moveit_sub_ = nh_.subscribe("/move_group/result", 1, &AgriControl::moveitCallback, this);
    goal_sub_ = nh_.subscribe(std::string("/" + prefix_ + "desired_robot_trajectory"), 1, &AgriControl::trajectoryCallback, this);

    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(std::string("/" + prefix_ + "eff_joint_traj_controller/command"), 1);
}

void AgriControl::setTrajNames(const sensor_msgs::JointState &js, const std::string &prefix)
{
    trajectory_goal_.joint_names.clear();
    for (uint i = 0; i < js.name.size(); i++)
        if (js.name[i].find(prefix) != std::string::npos)
            trajectory_goal_.joint_names.push_back(js.name[i]);
}

void AgriControl::initJointCallback(const sensor_msgs::JointState::ConstPtr &js)
{
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.resize(6);

    std::vector<double> tmp_v = {M_PI * 0.6, -M_PI * 0.6, 0, -M_PI * 0.5, -M_PI * 0.5, 0};
    for (uint i = 0; i < 6; i++)
        p.positions[i] = tmp_v[i];
    p.positions.back() = 0;
    p.time_from_start = ros::Duration(5);
    trajectory_goal_.points.push_back(p);
    ros::Duration(1).sleep();
    setTrajNames(*js, prefix_);
    trajectory_pub_.publish(trajectory_goal_);

    joint_sub_ = nh_.subscribe("/joint_states", 1, &AgriControl::jointCallback, this);
}

void AgriControl::jointCallback(const sensor_msgs::JointState::ConstPtr &js)
{

    for (uint i = 0; i < js->name.size(); i++)
        if (js->name[i].find(prefix_) != std::string::npos)
            joints_map_[js->name[i]] = js->position[i];
}

void AgriControl::sendTrajectory(const trajectory_msgs::JointTrajectory &traj, const std::string &prefix)
{
    trajectory_goal_.points.resize(traj.points.size());
    for (uint i = 0; i < trajectory_goal_.points.size(); i++)
    {
        trajectory_goal_.points[i].positions.resize(6);
        if (traj.points[i].time_from_start.toSec() == 0)
            trajectory_goal_.points[i].time_from_start = ros::Duration(i * 0.1);
        else
            trajectory_goal_.points[i].time_from_start = traj.points[i].time_from_start;

        static std::unordered_map<std::string, double>::iterator it;
        for (uint j = 0; j < traj.joint_names.size(); j++)
        {
            it = joints_map_.find(traj.joint_names[j]);
            if (it != joints_map_.end())
                it->second = traj.points[i].positions[j];
        }

        for (uint j = 0; j < trajectory_goal_.joint_names.size(); j++)
            trajectory_goal_.points[i].positions[j] = joints_map_[trajectory_goal_.joint_names[j]];
    }

    trajectory_pub_.publish(trajectory_goal_);
}

void AgriControl::trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &traj)
{
    sendTrajectory(*traj, prefix_);
}

void AgriControl::moveitCallback(const moveit_msgs::MoveGroupActionResult::ConstPtr &res)
{
    sendTrajectory(res->result.planned_trajectory.joint_trajectory, prefix_);
}

void AgriControl::spinner()
{
    ros::spinOnce();
}
