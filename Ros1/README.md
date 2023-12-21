# ERF Agri Challenge simulation framework

- Version 1.0.0

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

A repository for ERF Agri Challenge. Used for build, test, and deployment for the challenge.

- Maintainer status: maintained
- Maintainers
  - Andrea Pupa


</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [Overview](#overview)
- [Installation Steps](#installation-steps)
- [Usage](#usage)
- [Issues](#issues)

</div>
</div>

---

## Overview

<img src="images/erf_logo.png" width="200"/>  <img src="https://www.ros.org/imgs/logo-white.png" width="200"/>

- This repository provides framework and the URDF for the challenge for simulation 
- You can find information about the challenge on the <a href="https://erf2024.eu/challenges/">rule page</a>.
- The supported robot for this simulation is the Universal Robot UR5 endowed with the QbRobotics SoftHand

---

## Installation Steps

Everything should autoinstall with the command:
```
rosdep install --from-paths src --ignore-src -r -y
```

## Usage
You can run the demo using the launch file:

```
roslaunch agri_gazebo_scene scene.launch
```
The system is already integrate with MoveIt for planning purpose. This can be directly exploited with the argument use_moveit:
```
roslaunch agri_gazebo_scene scene.launch use_moveit:=true
```
Please, at the beginning deactivate and reactivate the MotionPlanning plugin on Rviz to align the robots with Gazebo.
Please note that when moving the left robot with the interface, there is a "green" robot that moves in strange positions. This is a visual bug that does not affect the simulation.

You can control the robot publishing on the topic
```
rostopic pub /desired_robot_trajectory ...
```

To close the hand you can publish on the topic:
```
rostopic pub /left_robot/set_gripper_state std_msgs/Bool "data: true"
rostopic pub /right_robot/set_gripper_state std_msgs/Bool "data: true"
```
And to reopen it:
```
rostopic pub /left_robot/set_gripper_state std_msgs/Bool "data: true"
rostopic pub /right_robot/set_gripper_state std_msgs/Bool "data: true"
```

Please note that the hand is an STL, thus it does not move in the scene. However closing and opening the gripper allow you to grasp the apples and move them inside the scene.

### Sensor topics:

Each cobot has a camera mounted on the base, which can be exploited to track the apples inside the environment.
To see the data, check the topics

```
/left/camera1/...
/right/camera1/...
```

## Issues:

Sometimes one or both manipulators initialize in a non correct configuration. While we fix that, we suggest to close and reopen everything.

If you encounter any issues, please contact me at my email address: andrea.pupa@unimore.it

## To Do List:
* Add actuation of softhand
* Solve bugs

