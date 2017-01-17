# jpmsc_mapping

FEUP - Robotics Course 2016/17

Author: João Pedro Milano Silva Cardoso - ee09063@fe.up.pt

Mapping robot using ROS Jade and the STDR Simulator.
Developed in Ubuntu 14.04

### Installation intructions:

* Configure the catkin workspace - http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* Install STDR from Github and not from apt packages (they are not up to date) into its own package in the workspace - http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator
* Create a package for the project
```
cd ~/catkin_ws/src
catkin_create_pkg jpmsc_mapping
```
* Copy the files to this folder

* Copy the files in jpmsc_mapping/stdr_files to the stdr_simulator package
```
jpmsc_mapping/stdr_files/stdr_launchers/launch --> stdr_simulator/stdr_launchers/launch
jpmsc_mapping/stdr_files/stdr_resources/maps --> stdr_simulator/stdr_resources/maps
jpmsc_mapping/stdr_files/stdr_resources/resources/robots --> stdr_simulator/stdr_resources/resources/robots
```

* Copy configuration file (vars.config) to the catkin workspace (~/catkin_ws/)
```
LL=1 for the LL method -> 360 lasers
LL=0 for the SL method -> 4 lasers
```

* Build the package with the following commands:
```
cd ~/catkin_ws
catkin_make
```

### To run the software

* Open a terminal and choose a robot and map (don't forget to set the robot mode in the configuration file):
```
roslaunch stdr_launchers jpmsc_mapping_1.launch - LL Robot Map 1
roslaunch stdr_launchers jpmsc_mapping_2.launch - LL Robot Map 1
roslaunch stdr_launchers jpmsc_mapping_3.launch - LL Robot Map 1
roslaunch stdr_launchers jpmsc_mapping_few_lasers_1.launch - SL Robot Map 1
roslaunch stdr_launchers jpmsc_mapping_few_lasers_2.launch - SL Robot Map 2
roslaunch stdr_launchers jpmsc_mapping_few_lasers_3.launch - SL Robot Map 3
```

* Open a second terminal and deploy the robot:
```
rosrun jpmsc_mapping robot_mapping_node robot0 laser_0
```
