# Shallow-Depth Insertion

## 1. Overview

This package is an implementation of **Shallow-Depth Insertion (SDI)**: a novel robotic manpulation technique suitable for assembling thin peg-like objects into a hole with a shallow depth, as can be seen in a cell phone battery insertion for example. Our technique features dexterous manipulation actions that combine into a complete insertion operation as seen in the animations below. This package is directly applicable to a simple hardware setting with the conventional parallel-jaw gripper installed on an industrial robot arm.

**Published Article**
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
- [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/) Industrial Robot Arm 

- [**Robotiq 2F-140**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper) Adaptive Parallel Jaw Gripper
- [**Robotiq FT300**](https://robotiq.com/products/ft-300-force-torque-sensor) Force Torque Sensor
- **Optional**: Webcam

<p align = "center">
<img src="files/exp_setting.png" width="400">
</p>

### 2.2 Software
- Our package was developed in [**Ubuntu 16.04**](http://releases.ubuntu.com/16.04/) and [**ROS Kinetic**](http://wiki.ros.org/ROS/Installation).
- [**MoveIt!**](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html): ROS robot planning framework
- [**ur_modern_driver**](https://github.com/ros-industrial/ur_modern_driver): ROS driver for UR10 robot controller from Universal Robots
- [**robotiq_2finger_grippers**](https://github.com/chjohnkim/robotiq_2finger_grippers.git): ROS driver for Robotiq Adaptive Grippers
- [**Eigen3 version 3.3**](https://askubuntu.com/questions/1068232/how-to-update-libeigen3-to-version-3-3-on-16-04-xenial)

## 3. Build on ROS
In your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/HKUST-RML/shallow_depth_insertion.git
cd ..
catkin_make
```

## 4. Run Shallow-Depth Insertion
### 4.1 Run in Simulation
SDI can be visualized in simulation through Gazebo and RVIZ. The robot arm motion is simulated but the object-robot contact interaction is currently not availble. 
To visualize the robot arm motion:
```
roslaunch shallow_depth_insertion ur10_robot.launch simulation:=true
roslaunch shallow_depth_insertion gripper.launch sim:=true
rosrun shallow_depth_insertion SDI_main.py
```

### 4.2 Run with Real Robot
If you are able to meet the basic hardware requirements (robot arm and gripper), you can execute SDI with the real robot:
```
roslaunch shallow_depth_insertion ur10_robot.launch simulation:=false
roslaunch shallow_depth_insertion gripper.launch
rosrun shallow_depth_insertion SDI_main.py
```

### 4.3 Changing execution parameters
The parameters of the SDI process can be specified in config/sdi_config.yaml.
The parameters are as follows:
- **Robot Parameter**
    - ***tcp_speed***: Robot tool center point (TCP) speed
- **Gripper Parameter**
    - ***tcp2fingertip***: Distance from TCP to gripper fingertip
    - ***finger_thickness***: Thickness of the robot finger
    - ***gripper_speed***: Gripper actuation speed between 0.013 and 0.100
    - ***gripper_force***: Gripper force between 0 and 100 (%)
- **Object Dimension**
    - ***object_thickness***: object thickness in meters
    - ***object_length***: object length in meters
- **SDI configuration**:
    - ***theta_0***: Angle between object and hole prior to regrasping
    - ***delta_0***: Distance from fingertip to object tip within gripper in meters
    - ***psi_regrasp***: Angle between object and upper finger after regrasping
    - ***theta_tilt***: Angle to tilt after regrasping
    - ***axis***: Axis and direction to rotate about using right-hand rule

## 5. Background
SDI is composed of three primitive operations: **tilt**, **regrasp**, and **tuck**. The three manipulation primitives are utilized to navigate the configuration space of the object-gripper-hole system, which is parameterized as follows:
- ***θ*** : Angle formed between the object and the hole surface
-  ***ψ*** : Angle formed between the object and the gripper upper finger
- ***γ*** : Ratio of the object enclosed by the gripper over the object length

<p align = "center">
<img src="files/config_space.JPG" width="250">
</p>

The three primitive operations navigate the configuration space as follows:

- **Tilt**: Changes ***θ*** by rotating the gripper-object system aobut the corner of the hole. No change in ***ψ*** and the distance between the two fingers.
- **Regrasp**: Changes ***ψ*** by rotating the gripper about the tip of the gripper upper finger. No change in ***θ***, but the distnace between the two fingers are coordinated accordingly. 
- **Tuck**: Changes ***θ*** and ***ψ*** by rotating the gripper about the tip of the gripper lower finger. 

***γ*** is fixed initially and remains fixed throughout the SDI process. 

<p align = "center">
<img src="files/tilt.jpg" width="250">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="files/regrasp.jpg" width="250">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="files/tuck.jpg" width="250">
</p>

## 6. Maintenance 
For any technical issues, please contact John Kim [chkimaa@connect.ust.hk](). 
