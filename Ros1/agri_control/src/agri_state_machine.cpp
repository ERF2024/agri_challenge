#include "agri_state_machine/agri_state_machine.h"

AgriStateMachine::AgriStateMachine(std::string prefix)
{
    gazebo_sub_ = nh_.subscribe("/gazebo/link_states", 1, &AgriStateMachine::gazeboCallback, this);

    std::string topic_name = "/" + prefix + "_robot/set_gripper_state";
    gripper_sub_ = nh_.subscribe(topic_name, 1, &AgriStateMachine::gripperCallback, this);
    gazebo_pub_ = nh_.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1);

    link_states_.name.resize(20 * 5);
    link_states_.pose.resize(20 * 5);

    prefix_ = prefix;
    machine_state_ = 0;
    gripper_state_ = 0; // The gripper is open
}

void AgriStateMachine::gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr &ls)
{
    int counter = 0;
    for (uint i = 0; i < ls->name.size(); i++)
    {
        size_t found1 = ls->name[i].find("plant_full");
        size_t found2 = ls->name[i].find("::a");
        size_t found3 = ls->name[i].find("wrist_3");
        if (found1 != std::string::npos && found2 != std::string::npos)
        {
            link_states_.name[counter] = ls->name[i];
            link_states_.pose[counter] = ls->pose[i];
            counter++;
        }
        else if (found3 != std::string::npos)
            robot_pose_ = ls->pose[i];
    }
}

void AgriStateMachine::gripperCallback(const std_msgs::Bool::ConstPtr &gs)
{
    gripper_state_ = int(gs->data);
}

double AgriStateMachine::distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return std::sqrt(std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2) + std::pow((p2.z - p1.z), 2));
}

bool AgriStateMachine::checkGrasp()
{
    for (uint i = 0; i < link_states_.pose.size(); i++)
    {
        if (distance(link_states_.pose[i].position, robot_pose_.position) < 0.07 && link_states_.pose[i].position.z > robot_pose_.position.z)
        {
            apple_idx_ = i;
            return true;
        }
    }
    return false;
}

void AgriStateMachine::moveApple()
{
    gazebo_msgs::LinkState link_state;
    link_state.link_name = link_states_.name[apple_idx_];
    link_state.pose = link_states_.pose[apple_idx_];
    link_state.pose.position = robot_pose_.position;
    link_state.reference_frame = "world";
    gazebo_pub_.publish(link_state);
}

void AgriStateMachine::spinner()
{
    ros::spinOnce();

    switch (machine_state_)
    {
    case 0:
        checkGrasp();
        // Gripper open: robot moving
        if (gripper_state_ == 1) // we need to check if the arm is close to the object or not
            if (checkGrasp())
                machine_state_ = 2;
            else
                machine_state_ = 1;
        break;

    case 1:
        // Gripper close and no object taken: you need to open
        ROS_INFO("Object Grasped");
        if (gripper_state_ == 0)
            machine_state_ = 0;
        break;

    case 2:
        // Gripper close and object taken: object moves together with the arm
        moveApple();
        if (gripper_state_ == 0)
            machine_state_ = 0;
        break;

    default:
        break;
    }
}