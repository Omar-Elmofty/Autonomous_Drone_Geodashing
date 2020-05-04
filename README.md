# Autonomous Drone Geodashing 

AER1217 - Autonomous UAS - Final project 

Project instructions, reference images and final report can be found in this [link](https://drive.google.com/open?id=17G31WV6U3oLcQBzALpfUpVduc_S7hC4u)

Link for [Demonstration video1](https://youtu.be/4ylh9HV_xOI) [Demonstration video2](https://youtu.be/-8_72JUhHaY) 

### Algorithms used: 

Landmark Detection: Color thresholding, Georeferencing

Landmark Classification: ORB feature detection and matching, RANSAC 

Motion Planning: RRT* followed by polynomial interpolation for path smoothing

Controller: PD controllers for pitch and roll, yaw controllers for z and yaw

## Requirements 

[ROS Kinetic](http://wiki.ros.org/kinetic)

Preferable environment: Ubuntu 16.04

## Simulator Environment Setup
To setup the simulator environment, please follow the simulator setup instructions [here](https://drive.google.com/open?id=12NddcTXf4h5ht1D1IKoDDKIvNOBbi42d)

After the simulator is setup, perform the following:

`$ cd ~/aer1217/labs/src`

Remove the existing `aer1217_ardrone_simulator` package, and replace it with the package in this repository

`$ cd ~/aer1217/labs`

`$ catkin_make`

Consult [ROS Wiki](http://wiki.ros.org/Documentation) if you encounter any issues setting up the package


## Obstacle and Landmark Localization 

You can use bag file provided in this [link](https://drive.google.com/open?id=17G31WV6U3oLcQBzALpfUpVduc_S7hC4u)

`$ rosbag play project.bag`

`$ rosrun aer1217_ardrone_simulator object_locator.py`

## Landmark Classification

`$ rosbag play project.bag`

`$ rosrun aer1217_ardrone_simulator landmark_classification.py`


## To run rrt_star planner:

Open rrt_star.py, and set the desired landmark visiting sequence in line 329

`$ roslaunch aer1217_ardrone_simulator ardrone_simulator.launch`


## Acknowledgments 
    
Course material and drone simulator from UTIAS AER1217 - Autonomous UAS course 

Full references list can be found in the [final report](https://drive.google.com/open?id=17G31WV6U3oLcQBzALpfUpVduc_S7hC4u)

