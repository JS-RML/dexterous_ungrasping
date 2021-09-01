#!/usr/bin/env python
import sys
import math
import rospy
import copy
import numpy as np
import tf
import moveit_commander
import helper
import globals #global variables 
globals.init("bat_0_KaHei.yaml") #initialize global variables
import motion_primitives
import yaml
import actionlib
import tilt
import regrasp
import tuck
import visualization
#import dynamixel

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
    with open(globals.config_path, 'r') as stream:
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
        tuck_angle = config['tuck']
        axis =  config['axis']
        object_thickness = config['object_thickness']
        object_length = config['object_length']
        tcp2fingertip = config['tcp2fingertip']
        sim = config['sim']
        table_height_wrt_world = 0.131-0.02 #battery box on table

        # Set TCP speed     
        group.set_max_velocity_scaling_factor(tcp_speed)
        
        # init_pose=[-0.344, 0.538, 0.458, -0.7071, -0.0, 0.7071, 0.0]
        # prepick_pose=[-0.2613, 0.6531, 0.3442, -0.7071, -0.0, 0.7071, 0.0]
        # pick_pose=[-0.2613, 0.6531, 0.2924, -0.7071, -0.0, 0.7071, 0.0]
        # tilt_pose=[-0.341, 0.413, 0.359, -0.653, -0.271, 0.653, 0.271]
        # prior_pose=[-0.436, 0.427, 0.360, -0.653, -0.271, 0.653, 0.271]
        # initial_pose=[-0.436, 0.437, 0.326, -0.653, -0.271, 0.653, 0.271]

        print "init pose"
        pose = [0.508, -0.82, table_height_wrt_world+tcp2fingertip+object_length-delta_0, 0.7071, 0, -0.7071, 0]
        motion_primitives.set_pose(pose)
        
        #Robotiq.goto(robotiq_client, pos=object_thickness+0.015, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        #rospy.sleep(0.5)
        #motion_primitives.set_pose(init_pose)
        
        #motion_primitives.set_pose(prepick_pose)
        #motion_primitives.set_pose(pick_pose)
        # print "extend palm"
        # dynamixel.set_length(152)
        # raw_input()
        print "init gripper position"
        Robotiq.goto(robotiq_client, pos=0.10, speed=config['gripper_speed'], force=config['gripper_force'], block=False)
        raw_input()
        print "close gripper"
        Robotiq.goto(robotiq_client, pos=object_thickness+0.001, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        raw_input()
        # print "init palm"
        # dynamixel.set_length(115)
        # rospy.sleep(0.5)
        # #motion_primitives.set_pose(prepick_pose)
        # #motion_primitives.set_pose(tilt_pose)
        # motion_primitives.set_pose(prior_pose)
        # rospy.sleep(10)
        # motion_primitives.set_pose(initial_pose)
        
        # read position from real robot. 
        p = group.get_current_pose().pose
        trans_tool0 = [p.position.x, p.position.y, p.position.z]
        rot_tool0 = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] 
        T_wg = tf.TransformerROS().fromTranslationRotation(trans_tool0, rot_tool0)
        P_g_center = [tcp2fingertip+object_length-delta_0, -object_thickness/2, 0, 1]
        P_w_center = np.matmul(T_wg, P_g_center)

        # P_w_center[2] = P_w_center[2] + 0.005
    
        # center = P_w_center[:3]
        center = [P_w_center[0], P_w_center[1], P_w_center[2]]

        #raw_input()

        motion_primitives.set_pose_relative([0, 0, 0.05])
        
        
        print "tilt"
        tilt.tilt([P_w_center[0], P_w_center[1], P_w_center[2]+0.05], axis, int(90-theta_0), tcp_speed)
        motion_primitives.set_pose_relative([0, 0.2, 0])

        raw_input()

        motion_primitives.set_pose_relative([0.005, -0.2, 0])
        motion_primitives.set_pose_relative([0, 0, -0.05])

        motion_primitives.set_pose_relative([0, -0.006, 0])

        raw_input()

        print "regrasp"
        # Regrasp
        #regrasp.palm_regrasp(np.multiply(axis, -1), int(psi_regrasp), tcp_speed)
        regrasp.regrasp(np.multiply(axis, -1), 32, tcp_speed)
        rospy.sleep(0.5)

        raw_input()

        print "tilt"
        # Tilt
        tilt.tilt(center, axis, 14, tcp_speed)
        rospy.sleep(0.5)
        
        print "slide"
        # Surface Slide
        motion_primitives.set_pose_relative([0, -0.006, 0])
        rospy.sleep(0.5)
        
        print "tuck"
        # Push-Tuck        
        # tuck.push_tuck(np.multiply(axis, -1), int(tuck_angle), 0.01, tcp_speed, 143)
        tuck.rotate_tuck(np.multiply(axis, -1), int(tuck_angle), 0.01, tcp_speed)
        rospy.sleep(0.5)
        
        motion_primitives.set_pose_relative([0, 0.01, 0])
        motion_primitives.set_pose_relative([0, 0, 0.1])

        #rospy.spin()
                
    except rospy.ROSInterruptException: pass
        
'''
# Robot parameters 
tcp_speed: 0.12

# Gripper parameters
tcp2fingertip: 0.275 # distance from tcp to gripper fingertip
opening_per_count: 0.00065 # gripper stroke opening per rPr count
finger_thickness: 0.005 
max_opening: 0.1523 # max stroke of gripper excluding finger thickness
gripper_speed: 0.1 # value between 0.013 and 0.100
gripper_force: 10 # value between 0 and 100

# Object dimension
object_thickness: 0.014 #0.01 # object thickness in meters
object_length: 0.049 # object length in meters

# Initial configuration
delta_0: 0.03 #0.0425 # distance from fingertip to object tip within gripper
theta_0: 45.0

# Intermediate configuration
psi_regrasp: 34.0
theta_tilt: 26
tuck: 21

# Action axis
axis: [1, 0, 0]

# Simulation
sim: 0
'''
