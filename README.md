# Shallow-Depth Insertion

## 1. Oveview

This package is an implementation of **Shallow-Depth Insertion (SDI)**: a novel robotic manpulation technique suitable for assembling thin peg-like objects into a hole with a shallow dpeth, as can be seen in a cell phone battery insertion for example. Our technique features dexterous manipulation actions that combine into a complete insertion operation as seen in the animations below. This package is directly applicable to a simple hardware setting with the conventional parallel-jaw gripper installed on an industrial robot arm.

**Related Paper**
- C. H. Kim and J. Seo, "[Shallow-Depth Insertion: Peg in Shallow Hole Through Robotic In-Hand Manipulation](https://ieeexplore.ieee.org/document/8598749)," in *IEEE Robotics and Automation Letters*, vol. 4, no. 2, pp. 383-390, April 2019.

    *If you use shallow-depth insertion for your application or research, please star this repo and cite our related paper.* [(BibTeX)](link)

<p align = "center">
<img src="files/card.gif" width="360" height="202"> <img src="files/phone.gif" width="360" height="202"> 
<img src="files/lego.gif" width="360" height="202"> <img src="files/battery.gif" width="360" height="202">
</p>

**Video Link:** [SDI](https://www.youtube.com/watch?v=Nka-sCzrcSs)

**Contributers**: [Chung Hee Kim](https://sites.google.com/view/chjohnkim/home), [Jungwon Seo](http://junseo.people.ust.hk/)  



## 2. Prerequisites

### 2.1 Hardware
- [**Universal Robot UR10**]((https://www.universal-robots.com/products/ur10-robot/)) Industrial Robot Arm
- [**Robotiq 2F-140**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) Adaptive Parallel Jaw Gripper
- [**Robotiq FT300**](https://robotiq.com/products/ft-300-force-torque-sensor) Force Torque Sensor
- **Optional**: Webcam

### 2.2 Software
- Our package was developed in [**Ubuntu 16.04**](http://releases.ubuntu.com/16.04/) and [**ROS Kinetic**](http://wiki.ros.org/ROS/Installation).
- [**MoveIt!**](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html): ROS robot planning framework
- [**ur_modern_driver**](https://github.com/ros-industrial/ur_modern_driver): ROS driver for UR10 robot controller from Universal Robots
- [**robotiq**](https://github.com/ros-industrial/robotiq): ROS driver for Robotiq Adaptive Grippers and Robotiq Force Torque Sensor



<p align = "center">
<img src="files/hardware_setup" width="360" height="202">
</p>


## 3. Build on ROS
In your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/chjohnkim/shallow_depth_insertion_v2.git
cd ..
catkin_make
```

## 4. Run Shallow-Depth Insertion

## 5. Background

## 6. Maintenance 
For any technical issues, please contact Chung Hee Kim [chkimaa@connect.ust.hk](). 
