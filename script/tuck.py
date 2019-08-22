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

    with open("/home/john/catkin_ws/src/shallow_depth_insertion_v2/config/sdi_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 
 
    # TODO: use pjg module
    #msg = rospy.wait_for_message('/CModelRobotInput', inputMsg.CModel_robot_input, timeout = None)
    #gripper_position = msg.gPO 
    gripper_position = 100 #TEMP DEBUG

    # gripper kinematics
    opening_at_zero = config['max_opening']-2*config['finger_thickness']
    gripper_opening = -config['opening_per_count']*gripper_position + opening_at_zero

    contact_B_e = [config['tcp2fingertip']-fingertip2contactB, gripper_opening/2.0, 0, 1]  
    contact_B_w = np.matmul(T_we, contact_B_e) 

    tilt.tilt(contact_B_w[:3], axis, angle, velocity)


if __name__ == '__main__':
    try:
        rospy.init_node('tilt', anonymous=True)  
        
        group.set_max_velocity_scaling_factor(1.0)
        motion_primitives.set_joint([0, -90, 90, 90, 90, 0])  
        p = group.get_current_pose().pose 
        tilt.tilt([p.position.x,p.position.y,p.position.z-0.1], [0,-1,0], 60, 0.5)
        rotate_tuck([0,1,0], 50, 0.1, 0.5)
    except rospy.ROSInterruptException: pass
