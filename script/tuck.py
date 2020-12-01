#!/usr/bin/env python
import sys
import math
import rospy
import copy
import numpy as np
import tf
import moveit_commander
import helper
import motion_primitives
import tilt
import yaml
import actionlib
import dynamixel
import globals as gbs

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface() 
group = moveit_commander.MoveGroupCommander("manipulator") 

def rotate_tuck(axis, angle, fingertip2contactB, velocity):
    '''Rotate tuck primitive motion of robot. 

    Parameters:
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tucking 
        fingertip2contactB (double): distance from fingertip to contact B in meters
        velocity (double): robot velocity between 0 and 1
    Returns:
    
    '''

    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)

    # TODO: use pjg module
    #msg = rospy.wait_for_message('/CModelRobotInput', inputMsg.CModel_robot_input, timeout = None)
    #gripper_position = msg.gPO 
    gripper_position = 255 #TEMP DEBUG
    
    #Robotiq.get_current_gripper_status(Robotiq())
    #Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        

    # gripper kinematics
    opening_at_zero = gbs.config['max_opening']-2*gbs.config['finger_thickness']
    gripper_opening = -gbs.config['opening_per_count']*gripper_position + opening_at_zero

    contact_B_e = [gbs.config['tcp2fingertip']-fingertip2contactB, -gripper_opening/2.0, 0, 1]  
    contact_B_w = np.matmul(T_we, contact_B_e) 

    tilt.tilt(contact_B_w[:3], axis, angle, velocity)

def active_rotate_tuck(axis, angle, fingertip2contactB, velocity, active_distance):
    '''Rotate tuck primitive motion of robot. 

    Parameters:
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tucking 
        fingertip2contactB (double): distance from fingertip to contact B in meters
        velocity (double): robot velocity between 0 and 1
    Returns:
    
    '''

    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)

    # TODO: use pjg module
    #msg = rospy.wait_for_message('/CModelRobotInput', inputMsg.CModel_robot_input, timeout = None)
    #gripper_position = msg.gPO 
    gripper_position = 255 #TEMP DEBUG
    
    #Robotiq.get_current_gripper_status(Robotiq())
    #Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        

    # gripper kinematics
    opening_at_zero = gbs.config['max_opening']-2*gbs.config['finger_thickness']
    gripper_opening = -gbs.config['opening_per_count']*gripper_position + opening_at_zero

    contact_B_e = [gbs.config['tcp2fingertip']-fingertip2contactB, -gripper_opening/2.0, 0, 1]  
    contact_B_w = np.matmul(T_we, contact_B_e) 

    tilt.active_tilt(contact_B_w[:3], axis, angle, velocity, active_distance)


def push_tuck(axis, angle, fingertip2contactB, velocity, tuck):
    '''Rotate tuck primitive motion of robot. 

    Parameters:
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tucking 
        fingertip2contactB (double): distance from fingertip to contact B in meters
        velocity (double): robot velocity between 0 and 1
    Returns:
    
    '''

    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)

    # TODO: use pjg module
    #msg = rospy.wait_for_message('/CModelRobotInput', inputMsg.CModel_robot_input, timeout = None)
    #gripper_position = msg.gPO 
    gripper_position = 255 #TEMP DEBUG
    
    #Robotiq.get_current_gripper_status(Robotiq())
    #Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        

    # gripper kinematics
    opening_at_zero = gbs.config['max_opening']-2*gbs.config['finger_thickness']
    gripper_opening = -gbs.config['opening_per_count']*gripper_position + opening_at_zero

    contact_B_e = [gbs.config['tcp2fingertip']-fingertip2contactB, -0.035/2.0, 0, 1]  
    contact_B_w = np.matmul(T_we, contact_B_e) 
    dynamixel.set_length(tuck)
    tilt.translate_tilt(contact_B_w[:3], axis, angle, velocity, 0.00)

def push_tuck2(axis, angle, fingertip2contactB, velocity, tuck):
    '''Rotate tuck primitive motion of robot. 

    Parameters:
        axis (list): 3-D vector of rotation axis (right-hand rule)
        angle (double): angle of tucking 
        fingertip2contactB (double): distance from fingertip to contact B in meters
        velocity (double): robot velocity between 0 and 1
    Returns:
    
    '''

    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)

    # TODO: use pjg module
    #msg = rospy.wait_for_message('/CModelRobotInput', inputMsg.CModel_robot_input, timeout = None)
    #gripper_position = msg.gPO 
    gripper_position = 255 #TEMP DEBUG
    
    #Robotiq.get_current_gripper_status(Robotiq())
    #Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        

    # gripper kinematics
    opening_at_zero = gbs.config['max_opening']-2*gbs.config['finger_thickness']
    gripper_opening = -gbs.config['opening_per_count']*gripper_position + opening_at_zero

    contact_B_e = [gbs.config['tcp2fingertip']-fingertip2contactB, -gripper_opening/2.0, 0, 1]  
    contact_B_w = np.matmul(T_we, contact_B_e) 
    dynamixel.set_length(tuck)
    tilt.translate_tilt(contact_B_w[:3], axis, angle, velocity, 0.003)
    
if __name__ == '__main__':
    try:

        rospy.init_node('tuck', anonymous=True)  
        group.set_max_velocity_scaling_factor(1.0)
        motion_primitives.set_joint([0, -90, 90, 90, 90, 0])  
        p = group.get_current_pose().pose 
        tilt.tilt([p.position.x,p.position.y,p.position.z-0.275], [0,-1,0], 60, 0.5)
        rotate_tuck([0,1,0], 50, 0.03, 0.1)
    except rospy.ROSInterruptException: pass
