![](https://i.imgur.com/ipB5pAy.jpg)
[NCRL website](http://ncrl.nctu.edu.tw/)
# Motion Prediction and Robust Tracking of a Dynamic and Temporarily-Occluded Target by an Unmanned Aerial Vehicle
## Introduction
A tracking controller for unmanned aerial vehicles (UAVs) is developed to track moving targets in the presence of occlusion. 
To the best of our knowledge, this is the first controller that can track moving targets based on a bounding box of the target detected by a deep neural network using the you-only-look-once (YOLO) method.

## Contents
* Demo
* Installation
    * Environment Setup
    * YOLO
    * Tracking Controller
* Implementation

## Demo
{%youtube YJ2ChIldr9A %}
{%youtube qz8sRHEVMaw %}

## Installation
### Environment Setup
* [Install ROS](http://wiki.ros.org/ROS/Installation)
* [Install Gazebo](https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html) 
Gazebo is often used with ROS, a toolkit/offboard API for automating vehicle control. 
* [Import Car model in Gazebo](https://github.com/osrf/car_demo)
The car is assumed to be the moving target to track. The moving target can be any detectable obeject pre-defined by the users. 
### YOLO
* [Install YOLO](https://github.com/n8886919/YOLO#Licence-Plate-Detection)
YOLO is utilized to obtain the image features and the relative angle from the bounding box of the moving target.
![](https://i.imgur.com/hAZaUgz.png)
### Tracking Controller
* [Install Tracking Controller](https://github.com/amychen1102/ncrl_motion_prediction_and_robust_tracking)

## Implementation
* Run px4 in Gazebo
```
roslaunch px4 mavros_posix_sitl.launch`
```
* Run YOLO
```
cd ~/Desktop/YOLO/car 
py video_node.py v11 --topic /drone/camera/image_raw 

```
* Run angle filter
```
roscd ukf_estimate/scripts/
python movingavg_filt.py
```
* Run UAV tracking controller 
```
roslaunch ukf_estimate target_estimate_angle.launch
```
While UAV takes off
Step 1. Click buttom 3 - activate UKF
Step 2. Click buttom 2 - activate tracking controller

* Run QP (quadratic programming)
```
roscd ukf_estimate/scripts/
python target_trajectory_qp_3deg.py -r 80 -l 600 -w 0.1 
```
(-r : loop rate, -l : window lengh, -w : regulator weight)
