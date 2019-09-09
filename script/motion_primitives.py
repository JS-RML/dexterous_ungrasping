#!/usr/bin/env python
import sys
import math
import rospy
import copy
import moveit_commander 
import yaml

moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)

with open("/home/john/catkin_ws/src/shallow_depth_insertion/config/sdi_config.yaml", 'r') as stream:
    try:
        config = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
tcp_speed = config['tcp_speed']
group.set_max_velocity_scaling_factor(tcp_speed)

def set_pose(pose):
    '''Set robot pose.  

    Parameters:
        pose (list): robot pose [x, y, z, qx, qy, qz, qw]
    Returns:
    
    '''
    pose_target = group.get_current_pose() 
    pose_target.pose.position.x = pose[0]
    pose_target.pose.position.y = pose[1]
    pose_target.pose.position.z = pose[2]
    pose_target.pose.orientation.x = pose[3]
    pose_target.pose.orientation.y = pose[4]
    pose_target.pose.orientation.z = pose[5]
    pose_target.pose.orientation.w = pose[6]
    group.set_pose_target(pose_target) 
    plan = group.plan() 
    group.go(wait=True) 

def set_joint(joint):
    '''Set robot joint.  

    Parameters:
        joint (list): robot joints in degrees
    Returns:
    
    '''
    j = group.get_current_joint_values() 
    j[0] = math.radians(joint[0])
    j[1] = math.radians(joint[1])
    j[2] = math.radians(joint[2])
    j[3] = math.radians(joint[3])
    j[4] = math.radians(joint[4])
    j[5] = math.radians(joint[5])
    group.set_joint_value_target(j)
    plan = group.plan() 
    group.go(wait=True)

def set_joint_relative(joint_relative):
    '''Set relative robot joint.  

    Parameters:
        joint_relative (list): relative robot joints in degrees
    Returns:
    
    '''
    j = group.get_current_joint_values() 
    j[0] += math.radians(joint_relative[0])
    j[1] += math.radians(joint_relative[1])
    j[2] += math.radians(joint_relative[2])
    j[3] += math.radians(joint_relative[3])
    j[4] += math.radians(joint_relative[4])
    j[5] += math.radians(joint_relative[5])
    group.set_joint_value_target(j) 
    plan = group.plan() 
    group.go(wait=True)  

def set_pose_relative(relative_position):
    '''Set relative robot position.  

    Parameters:
        relative_position (list): relative robot position [x, y, z]
    Returns:
    
    '''
    pose_target = group.get_current_pose() 
    pose_target.pose.position.x += relative_position[0]
    pose_target.pose.position.y += relative_position[1]
    pose_target.pose.position.z += relative_position[2]
    group.set_pose_target(pose_target) 
    plan = group.plan() 
    group.go(wait=True) 

def linear_path(vector, velocity):
    '''Set relative robot position via linear trajectory.  

    Parameters:
        vector (list): relative robot position [x, y, z]
        velocity (float): velocity scale 0~1
    Returns:
    
    '''
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    waypoints = [] 
    for t in range(1,11):
        t = t*0.1
        pose_target.position.x = x_1 + vector[0]*t
        pose_target.position.y = y_1 + vector[1]*t
        pose_target.position.z = z_1 + vector[2]*t
        waypoints.append(copy.deepcopy(pose_target))     
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) 
    retimed_plan= group.retime_trajectory(robot.get_current_state(), plan, velocity) #parameter that changes velocity
    group.execute(retimed_plan)

if __name__ == '__main__':
    try:
        rospy.init_node('motion_primitives', anonymous=True)
        group.set_max_velocity_scaling_factor(1.0)
        set_joint([0, -90, 90, 0, 90, 0])
        set_pose_relative([-0.2, 0, 0]) 
        linear_path([0.9, 0, 0], 0.5)
    except rospy.ROSInterruptException: pass