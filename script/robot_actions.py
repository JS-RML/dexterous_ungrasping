#!/usr/bin/env python
import sys
import math
import time
import rospy
import copy
import numpy as np
import tf
import moveit_commander
import helper
import motion_primitives
import yaml
import actionlib
import visualization
import dynamixel 

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface() 
group = moveit_commander.MoveGroupCommander("manipulator") 

#import std_msgs.msg 
#import geometry_msgs.msg 
#import roslib; roslib.load_manifest('robotiq_c_model_control')
#from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
#from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
#from apriltags_ros.msg import * 
#from geometry_msgs.msg import WrenchStamped
#from std_msgs.msg import *
#from rospy import init_node, is_shutdown

def grab_object(distance):
    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    Robotiq.goto(robotiq_client, pos=object_thickness+0.015, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
    rospy.sleep(0.5)
    motion_primitives.set_pose_relative([0, 0, -distance])
    Robotiq.goto(robotiq_client, pos=object_thickness+0.002, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
    rospy.sleep(0.5)
    motion_primitives.set_pose_relative([0, 0, distance])
        

def goto_aruco(tag_id, offset_x, offset_y, offset_z):
    tf_listener = tf.TransformListener()
    tag_frame_name = 'tag_'+str(tag_id)
    tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
    (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) 
    motion_primitives.set_pose([trans_tag[0]+offset_x, trans_tag[1]+offset_y, trans_tag[2]+offset_z, -0.7071, -0.0, 0.7071, 0.0])
        
    print trans_tag, rot_tag
      
#############################################################################################################################################################################################################
####____SENSING____####
#############################################################################################################################################################################################################


###___FORCE SEEK___###
def force_seek2(axis_world, distance, force_direction, sensitivity, final_offset, vel):
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    if axis_world is 'x':
        x_2 = x_1 + distance
        y_2 = y_1
        z_2 = z_1
    if axis_world is 'y':
        x_2 = x_1
        y_2 = y_1 + distance
        z_2 = z_1
    if axis_world is 'z':
        x_2 = x_1
        y_2 = y_1
        z_2 = z_1 + distance
    direction_vector = [x_2-x_1, y_2-y_1, z_2-z_1]
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    t = 0 # counter/increasing variabe for the parametric equation of straight line      
    while t <= 1.01:
        pose_target.position.x = x_1 + direction_vector[0]*t
        pose_target.position.y = y_1 + direction_vector[1]*t
        pose_target.position.z = z_1 + direction_vector[2]*t
        t += resolution 
        
        waypoints.append(copy.deepcopy(pose_target))
         
    del waypoints[:1]
    
    forceseek_asyncExecute_waypoints(waypoints)

    rospy.sleep(0.5)
    wrench = rospy.wait_for_message('/robotiq_force_torque_wrench', WrenchStamped, timeout = None)
    
    if force_direction is 'x':
        force_initial = wrench.wrench.force.x
    if force_direction is 'y':
        force_initial = wrench.wrench.force.y
    if force_direction is 'z':
        force_initial = wrench.wrench.force.z
    #print 'threshold = ', math.fabs(force_initial)+sensitivity
    print sensitivity
    force = 0
    i = 0
    while i is not 4:
        wrench = rospy.wait_for_message('/robotiq_force_torque_wrench', WrenchStamped, timeout = None)        
        if force_direction is 'z':
            force = wrench.wrench.force.z
        if force_direction is 'y':
            force = wrench.wrench.force.y
        if force_direction is 'x':
            force = wrench.wrench.force.x 
        #print math.fabs(force) 
        if math.fabs(force) > math.fabs(force_initial)+sensitivity:
            i += 1
        print math.fabs(force) - math.fabs(force_initial)
    print 'STOP'  
    group.stop()
    #relative_pose_target(axis_world, final_offset)
    
    tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
    (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
    x_1 = trans_eelink[0]
    y_1 = trans_eelink[1]
    z_1 = trans_eelink[2]
    if axis_world is 'x':
        x_2 = x_1 + final_offset
        y_2 = y_1
        z_2 = z_1
    if axis_world is 'y':
        x_2 = x_1
        y_2 = y_1 + final_offset
        z_2 = z_1
    if axis_world is 'z':
        x_2 = x_1
        y_2 = y_1
        z_2 = z_1 + final_offset
    direction_vector = [x_2-x_1, y_2-y_1, z_2-z_1]
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    t = 0 # counter/increasing variabe for the parametric equation of straight line      
    while t <= 1.01:
        pose_target.position.x = x_1 + direction_vector[0]*t
        pose_target.position.y = y_1 + direction_vector[1]*t
        pose_target.position.z = z_1 + direction_vector[2]*t
        t += resolution 
        
        waypoints.append(copy.deepcopy(pose_target))
         
    del waypoints[:1]
    plan_execute_waypoints(waypoints)
 

###___TRACK APRILTAG___###
##Detect april tag and make the end-effector point directly at the origin of the april tag frame
def point_apriltag(tag_id, tag_frame_name):
    rospy.sleep(1)
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout = None) 
    direction_vector = [0,0,0]
    direction_vector_normalized = [0, 0, 0]
    orthogonal_dot = [0, 0, 0]
    detection = False
    x = 0
    while x < 20:
        if msg.detections[x].id is tag_id:
            detection = True
            break
        x += 1
    if detection is True:
    #if msg.detections[0].id is tag_id or msg.detections[1].id is tag_id or msg.detections[2].id is tag_id or msg.detections[3].id is tag_id or msg.detections[4].id is tag_id or msg.detections[5].id is tag_id or msg.detections[6].id is tag_id or msg.detections[7].id is tag_id: 
        rate = rospy.Rate(10)
        tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
        (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
        tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
        (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
        direction_vector[0] = trans_tag[0] - trans_eelink[0]
        direction_vector[1] = trans_tag[1] - trans_eelink[1]
        direction_vector[2] = trans_tag[2] - trans_eelink[2]
        
        #normalize direction_vector to unit length
        length = math.sqrt(direction_vector[0]*direction_vector[0]+direction_vector[1]*direction_vector[1]+direction_vector[2]*direction_vector[2])
        direction_vector_normalized[0] = direction_vector[0] / length
        direction_vector_normalized[1] = direction_vector[1] / length
        direction_vector_normalized[2] = direction_vector[2] / length
        

        #orthgonal by cross product with a standard vector e_y
        e_y = [0, 1, 0] # this parameter needs to be changed according to the general workspace of the robot 
        orthogonal_standard = numpy.cross(direction_vector_normalized, e_y)
        length = math.sqrt(orthogonal_standard[0]*orthogonal_standard[0]+orthogonal_standard[1]*orthogonal_standard[1]+orthogonal_standard[2]*orthogonal_standard[2])
        orthogonal_standard[0] = orthogonal_standard[0] / length
        orthogonal_standard[1] = orthogonal_standard[1] / length
        orthogonal_standard[2] = orthogonal_standard[2] / length 
        
        #orthogonal by cross product
        orthogonal_cross = numpy.cross(direction_vector_normalized, orthogonal_standard)
 
        #Fill the Rotation matrix 
        I = tf.transformations.identity_matrix()
        I[0,0] = direction_vector_normalized[0]
        I[1,0] = direction_vector_normalized[1]
        I[2,0] = direction_vector_normalized[2]
        I[0,1] = orthogonal_standard[0]
        I[1,1] = orthogonal_standard[1]
        I[2,1] = orthogonal_standard[2]
        I[0,2] = orthogonal_cross[0]
        I[1,2] = orthogonal_cross[1]
        I[2,2] = orthogonal_cross[2]
        I[0,3] = trans_eelink[0]
        I[1,3] = trans_eelink[1]
        I[2,3] = trans_eelink[2]
        quat_from_mat = tf.transformations.quaternion_from_matrix(I)
        assign_pose_target(trans_eelink[0], trans_eelink[1], trans_eelink[2], quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
    else:
        print tag_frame_name, ' not found'

###___TRACK APRIL TAG___###
## Detects april tag and moves to the desired position w.r.t. the detected apriltag 
# Specify the values of offset_x, offset_y, offset_z to adjust the final position of the end-effector tip 
def track_apriltag(tag_id, tag_frame_name, offset_x, offset_y, offset_z):
    rospy.sleep(1)
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
 
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout = None)
    detection = False
    x = 0
    while x < 20:
        if msg.detections[x].id is tag_id:
            detection = True
            break
        x += 1
    if detection is True:
        tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
        (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
        tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
        (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
        i = 1
        while i is 1:

            pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
            waypoints = []
            waypoints.append(pose_target)

            tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
            (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
            tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
            (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
        
    
            x_1 = trans_eelink[0]
            y_1 = trans_eelink[1]
            z_1 = trans_eelink[2]
            x_2 = trans_tag[0]+offset_x     
            y_2 = trans_tag[1]+offset_y 
            z_2 = trans_tag[2] + offset_z
            v = [x_2-x_1, y_2-y_1, z_2-z_1]
            
            t = 0 # counter/increasing variabe for the parametric equation of straight line      
            while t <= 1.01:
                pose_target.position.x = x_1 + v[0]*t
                pose_target.position.y = y_1 + v[1]*t
                pose_target.position.z = z_1 + v[2]*t    
                store_x = x_1 + v[0]*t
                store_y = y_1 + v[1]*t
                store_z = z_1 + v[2]*t
   
                direction_vector = [0,0,0]
                direction_vector_normalized = [0, 0, 0]
                orthogonal_dot = [0, 0, 0]
                    
                direction_vector[0] = x_2 - store_x 
                direction_vector[1] = y_2 - store_y 
                direction_vector[2] = z_2 - store_z - offset_z
        
                #normalize direction_vector to unit length
                length = math.sqrt(direction_vector[0]*direction_vector[0]+direction_vector[1]*direction_vector[1]+direction_vector[2]*direction_vector[2])
                direction_vector_normalized[0] = direction_vector[0] / length
                direction_vector_normalized[1] = direction_vector[1] / length
                direction_vector_normalized[2] = direction_vector[2] / length
        
                #orthgonal by cross product with a standard vector e_y
                e_y = [0, 1, 0] # this parameter needs to be changed according to the general workspace of the robot 
                orthogonal_standard = numpy.cross(direction_vector_normalized, e_y)
                length = math.sqrt(orthogonal_standard[0]*orthogonal_standard[0]+orthogonal_standard[1]*orthogonal_standard[1]+orthogonal_standard[2]*orthogonal_standard[2])
                orthogonal_standard[0] = orthogonal_standard[0] / length
                orthogonal_standard[1] = orthogonal_standard[1] / length
                orthogonal_standard[2] = orthogonal_standard[2] / length 
        
                #orthogonal by cross product
                orthogonal_cross = numpy.cross(direction_vector_normalized, orthogonal_standard)

                #Fill the Rotation matrix 
                I = tf.transformations.identity_matrix()
                I[0,0] = direction_vector_normalized[0]
                I[1,0] = direction_vector_normalized[1]
                I[2,0] = direction_vector_normalized[2]
                I[0,1] = orthogonal_standard[0]
                I[1,1] = orthogonal_standard[1]    
                I[2,1] = orthogonal_standard[2]    
                I[0,2] = orthogonal_cross[0]
                I[1,2] = orthogonal_cross[1]
                I[2,2] = orthogonal_cross[2]
                I[0,3] = store_x    
                I[1,3] = store_y
                I[2,3] = store_z
                quat_from_mat = tf.transformations.quaternion_from_matrix(I)    
                
                pose_target.orientation.x = quat_from_mat[0]
                pose_target.orientation.y = quat_from_mat[1]
                pose_target.orientation.z = quat_from_mat[2]
                pose_target.orientation.w = quat_from_mat[3]
                waypoints.append(copy.deepcopy(pose_target))
                 
                t += resolution 
    
            del waypoints[:1]
            plan_execute_waypoints(waypoints)
        
            tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
            (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
            tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
            (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
            i = 2
    else:
        print tag_frame_name, ' not found.'  
  
def plan_execute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan)

def forceseek_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, 0.002) #parameter that changes velocity
    group.execute(plan, wait = False)

if __name__ == '__main__':

    try:
        print "robot_actions"        
    except rospy.ROSInterruptException: pass
