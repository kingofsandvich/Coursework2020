# Coursework2020
## Requirements
This ROS Kinetic package requires pr2, gazebo, moveit and CRUMB packages installed.
Contains simulation for multi-agent simulation for [map-spatial planner](https://github.com/ihmcrobotics/ihmc_ros_core/tree/develop/ihmc_ros_java_adapter).

## Launch

To launch simulation the following commands:
`roslaunch map_simulation task0.launch`  to run gazebo map coresponding to spatial task 0;
`rosrun map_simulation planner_adapter.py` to run script translating `solution.txt` file to robots' actions in simulation started.

`planner_adapter.py` accepts names of objects and agents, and relative path to solution file as arguments.
