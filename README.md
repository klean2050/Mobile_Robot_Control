# Differential-Drive Mobile Robot Control Project

Project in NTUA (2018-2019) for the Robotics II course of 8th semester
### Kleanthis Avramidis,  Agelos Kratimenos, Ilias Panagiotaras,  Alexandros Barberis

## Description
The current repository contains an implementation of a wall following and a localization algorithm in the form of ROS packages. The algorithms were tested on a *Raspberry Pi 3 Model B* with *Ubuntu 16.04*. The packages were developed and tested for ROS Kinetic. Supplementary packages for the sonars, the motors, the IMU and a random walking algorithm are also included. Below are the descriptions of the implemented packages in python:

1.**wall_following:** 
The algorithm consists of four states based on the sonar's input data: 
* State 0 (wandering): The robot is moving at a constant linear and angular velocity in its effort to reach the first wall.
* State 1 (turning): After detecting the first wall, the robot turns at a specific angle in order to align itself with the wall. The robot maintains the linear velocity constant, while the angular one is increased.
* State 2 (wall following): As the robot has aligned itself with the wall, it begins to ''follow'' it, while keeping the desired distance from it, with the use of a PD controller. Specifically, the controller regulates the distance between the center of the robot and the wall, as well as its angle in respect to the wall.
* State 3 (stoping): The robot comes to a halt when a specific number of turns has been accomplished.

![wall following algorithm flow chart](https://i.ibb.co/LRYCy5t/Screenshot-2.png)

2.**localization:** An implementation of an Extended Kalman Filter. For the prediction model we use the classical equations of motion, the linear acceleration and the angular velocity that are provided by the IMU. For the measurement model the values provided by the sonars and the angle measurements by the IMU. Provided that we have prior knowledge of the map the robot explores, we use the angle to specify which wall the robot is facing and with the sonar readings we calculate the robot's exact position.

![localization figure](https://i.ibb.co/ggkgW0G/Screenshot-3.png)

3.**read_sonars:** This package reads the sonar values and publishes them to the corresponding topics.

4.**dc_motor_driver:** This package enables the use of the dc motors.

5.**i2c_imu:** This package reads the values obtained from the inertial measurement unit attached to the robot (accelaration, orientation) and publishes them to the corresponding topics. The IMU, firstly, needs to be calibrated. 

6.**random_walk:** The robot moves at a constant linear velocity and turns when a wall is detected by the sonars. It is used in conjuction with the localization package.

## Usage
1. Clone the repository.
2. Create a ROS (Kinetic) workspace.
3. Move the packages in the catkin_ws/src folder.
4. Run catkin_make in catkin_ws folder.
5. Run ```roslaunch dc_motor_driver dc_motor_driver.launch```.
6. Run ```roslaunch read_sonars read_sonars.launch```
7. For the wall following algorithm all you need is to run ```./catkin_ws/src/wall_following/scripts/follower_new.py```. 
For the localization algorithm you first need to run ```rosrun random_walk walker```, ```rosrun i2c_imu i2c_imu_node ``` 
and finally ```./catkin_ws/src/localization/scripts/localization.py```.

## Observations
* The sonar readings are not particularly accurate, as their values flactuate. That's why the above algorithms may fail to work as expected at some trials.
* Moreover, the sonars may fail to accurately measure a distance from a wall, when its thickness is low. That may cause the robot to take very sharp or even very wide turns.
* Due to the sonar errors the exact position may not be estimated very accurately, although the right quadrant in the map can be specified.



***This project is licensed under the terms of the MIT license.***
