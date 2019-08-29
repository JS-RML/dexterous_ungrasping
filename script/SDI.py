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
import yaml
import actionlib
import tilt
import regrasp
import tuck
import visualization


from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

rospy.init_node('SDI', anonymous=True)  
action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface() 
group = moveit_commander.MoveGroupCommander("manipulator") 
   
if __name__ == '__main__':
    with open("/home/john/catkin_ws/src/shallow_depth_insertion/config/sdi_config.yaml", 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    try:
        tcp_speed = config['tcp_speed']
        theta_0 = config['theta_0']
        delta_0 = config['delta_0']
        psi_regrasp = config['psi_regrasp']
        theta_tilt = config['theta_tilt']
        axis =  config['axis']
        object_thickness = config['object_thickness']
        object_length = config['object_length']
        tcp2fingertip = config['tcp2fingertip']
        table_height_wrt_world = -0.02

        group.set_max_velocity_scaling_factor(tcp_speed)
        Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False) 
        motion_primitives.set_joint([0, -90, 90, 90, 90, 0])  
        init_pose = [0.6566, 0.1638, table_height_wrt_world + object_length + tcp2fingertip - delta_0, -0.7071, 0.000, 0.7071, 0.000]
        
        motion_primitives.set_pose(init_pose)
        p = group.get_current_pose().pose

        '''
        #TEST GRIPPER LENGTH
        pos_initial = [p.position.x, p.position.y, p.position.z]
        ori_initial = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        T_we = tf.TransformListener().fromTranslationRotation(pos_initial, ori_initial) 
        tcp2fingertip = config['tcp2fingertip']
        contact_A_e = [tcp2fingertip, -object_thickness/2, 0, 1] 
        contact_A_w = np.matmul(T_we, contact_A_e) 
        visualization.visualizer(contact_A_w[:3], "s", 0.005, 2) #DEBUG
        #TEST GRIPPER LENGTH
        '''

        center = [p.position.x,p.position.y,p.position.z - tcp2fingertip - object_length + delta_0]
        visualization.visualizer(center, "s", 0.01, 0)
        rospy.sleep(0.5)
        tilt.tilt(center, axis, int(90-theta_0), tcp_speed)
        regrasp.regrasp(np.multiply(axis, -1), int(psi_regrasp), tcp_speed)
        tilt.tilt(center, axis, int(theta_tilt), tcp_speed)
        tuck.rotate_tuck(np.multiply(axis, -1), int(theta_0-theta_tilt), 0.03, tcp_speed)
        rospy.spin()
        
    except rospy.ROSInterruptException: pass
