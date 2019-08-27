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
import tilt
import yaml
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface() 
group = moveit_commander.MoveGroupCommander("manipulator") 

def regrasp(axis, angle, velocity):
    with open("/home/john/catkin_ws/src/shallow_depth_insertion_v2/config/sdi_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    pose_target = group.get_current_pose().pose
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 
    tcp2fingertip = config['tcp2fingertip']
    contact_A_e = [tcp2fingertip, -config['object_thickness']/2, 0, 1] #TODO: depends on axis direction
    contact_A_w = np.matmul(T_we, contact_A_e) 

    # Interpolate orientation poses via quaternion slerp
    q = helper.axis_angle2quaternion(axis, angle)
    ori_target = tf.transformations.quaternion_multiply(q, ori_initial)    
    ori_waypoints = helper.slerp(ori_initial, ori_target, np.arange(1.0/angle , 1.0+1.0/angle, 1.0/angle)) 

    theta_0 = config['theta_0']
    waypoints = []
    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
        
    for psi in range(1, angle+1):
        # Calculate width
        a = config['delta_0'] * math.cos(math.radians(psi))
        b = config['delta_0'] * math.sin(math.radians(psi))
        c = config['object_thickness'] * math.cos(math.radians(psi))
        d = config['object_thickness'] * math.sin(math.radians(psi))
        opposite = a - d
        width = b + c

        # Calculate position 
        if theta_0 + psi <= 90:
            hori =  math.fabs(tcp2fingertip*math.cos(math.radians(theta_0 + psi))) + math.fabs((width/2.0)*math.sin(math.radians(theta_0+psi)))
            verti =  math.fabs(tcp2fingertip*math.sin(math.radians(theta_0 + psi))) - math.fabs((width/2.0)*math.cos(math.radians(theta_0+psi)))
        else:
            hori = -math.fabs(tcp2fingertip*math.sin(math.radians(theta_0 + psi-90))) + math.fabs((width/2.0)*math.cos(math.radians(theta_0+psi-90)))
            verti = math.fabs(tcp2fingertip*math.cos(math.radians(theta_0 + psi-90))) + math.fabs((width/2.0)*math.sin(math.radians(theta_0+psi-90)))

        if axis[0] > 0:
            pose_target.position.y = contact_A_w[1] + hori
            pose_target.position.z = contact_A_w[2] + verti
            #print "CASE 1"
        elif axis[0] < 0:
            pose_target.position.y = contact_A_w[1] - hori
            pose_target.position.z = contact_A_w[2] + verti
            #print "CASE 2"
        elif axis[1] > 0:
            pose_target.position.x = contact_A_w[0] - hori
            pose_target.position.z = contact_A_w[2] + verti
            #print "CASE 3"
        elif axis[1] < 0:
            pose_target.position.x = contact_A_w[0] + hori
            pose_target.position.z = contact_A_w[2] + verti
            #print "CASE 4"

        pose_target.orientation.x = ori_waypoints[psi-1][0]
        pose_target.orientation.y = ori_waypoints[psi-1][1]
        pose_target.orientation.z = ori_waypoints[psi-1][2]
        pose_target.orientation.w = ori_waypoints[psi-1][3]
        waypoints.append(copy.deepcopy(pose_target))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) 
    retimed_plan = group.retime_trajectory(robot.get_current_state(), plan, velocity) 
    group.execute(retimed_plan, wait=False)

    
    opening_at_zero = config['max_opening']-2*config['finger_thickness']
    psi = 0
    while psi < angle:
        pose = group.get_current_pose().pose
        q_current = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        psi = 2*math.degrees(math.acos(np.dot(q_current, ori_initial)))
        if psi > 100:
            psi = -(psi-360)
        a = config['delta_0'] * math.cos(math.radians(psi))
        b = config['delta_0'] * math.sin(math.radians(psi))
        c = config['object_thickness'] * math.cos(math.radians(psi))
        d = config['object_thickness'] * math.sin(math.radians(psi))
        opposite = a - d
        width = b + c
        #pos = int((opening_at_zero - width)/config['opening_per_count'])
        Robotiq.goto(robotiq_client, pos=width, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        psi = round(psi, 2)
   
if __name__ == '__main__':
    try:
        rospy.init_node('regrasp', anonymous=True)  
        group.set_max_velocity_scaling_factor(1.0)
        motion_primitives.set_joint([0, -90, 90, 90, 90, 0])  
        p = group.get_current_pose().pose 
        tilt.tilt([p.position.x,p.position.y,p.position.z-0.275], [0,-1,0], 60, 0.5)
        regrasp([0,1,0], 90, 0.1)
        
    except rospy.ROSInterruptException: pass
