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
import robot_actions

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

from geometry_msgs.msg import WrenchStamped # TODO: import /wrench
def print_transition(transition_label):
    msg = rospy.wait_for_message('/robotiq_ft_wrench', WrenchStamped, timeout = None) # TODO: /wrench
    print transition_label, ": ",  msg.header.stamp, "secs" # TODO: msg.header.stamp.secs

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
        tuck_angle = config['tuck']
        axis =  config['axis']
        object_thickness = config['object_thickness']
        object_length = config['object_length']
        tcp2fingertip = config['tcp2fingertip']
        sim = config['sim']
        table_height_wrt_world = -0.02
        
        # Set TCP speed     
        group.set_max_velocity_scaling_factor(tcp_speed)
        
        
        pre_pick_pose = [-0.092, 0.592, 0.416, 0.7071, -0.0, -0.7071, 0.0]
        pick_pose = [-0.092, 0.592, 0.282, 0.7071, -0.0, -0.7071, 0.0]
        init_pose=[-0.344, 0.538, 0.458, -0.7071, -0.0, 0.7071, 0.0]
        regrasp_pose=[-0.344, 0.538, 0.350, -0.7071, -0.0, 0.7071, 0.0]
        
        Robotiq.goto(robotiq_client, pos=0.04, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        rospy.sleep(0.5)
                
        motion_primitives.set_pose(pre_pick_pose)
        rospy.sleep(0.5)
        motion_primitives.set_pose(pick_pose)
        rospy.sleep(1)
        Robotiq.goto(robotiq_client, pos=object_thickness+0.002, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        rospy.sleep(1)
        motion_primitives.set_pose(pre_pick_pose)
        #Robotiq.goto(robotiq_client, pos=object_thickness+0.003, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        
        
        motion_primitives.set_pose(regrasp_pose)
        
        #for i in range(11):
        #    rospy.sleep(1)
        #    print 10-i

        # read position from real robot. 
        
        p = group.get_current_pose().pose
        trans_tool0 = [p.position.x, p.position.y, p.position.z]
        rot_tool0 = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] 
        T_wg = tf.TransformerROS().fromTranslationRotation(trans_tool0, rot_tool0)
        P_g_center = [tcp2fingertip+object_length-delta_0, 0, 0, 1]
        P_w_center = np.matmul(T_wg, P_g_center)
        center = [P_w_center[0], P_w_center[1], P_w_center[2]]
        
        # Tilt
        tilt.tilt(center, axis, int(90-theta_0), tcp_speed)
        rospy.sleep(5)

        # Regrasp
        print_transition("REGRASP START")
        regrasp.regrasp(np.multiply(axis, -1), int(psi_regrasp), tcp_speed)
        rospy.sleep(2)
        print_transition("REGRASP FINISHED")
       
        # Tilt
        print_transition("TILT START")
        tilt.tilt(center, axis, int(theta_tilt), tcp_speed)
        print_transition("TILT FINISHED")
        rospy.sleep(2)

        
        # Place
        print_transition("TUCK START")
        tuck.rotate_tuck(np.multiply(axis, -1), int(tuck_angle), 0.035, tcp_speed)
        print_transition("TUCK FINISHED")
        rospy.sleep(1)

        print_transition("RELEASING CONTACT")
        motion_primitives.set_pose_relative([0,0,0.05])
        
        #rospy.spin()
        
    except rospy.ROSInterruptException: pass
        
'''
# Robot parameters 
tcp_speed: 0.05

# Gripper parameters
tcp2fingertip: 0.28 # distance from tcp to gripper fingertip
opening_per_count: 0.00065 # gripper stroke opening per rPr count
finger_thickness: 0.005 
max_opening: 0.1523 # max stroke of gripper excluding finger thickness
gripper_speed: 0.1 # value between 0.013 and 0.100
gripper_force: 10 # value between 0 and 100

# Object dimension
object_thickness: 0.003 #0.01 # object thickness in meters
object_length: 0.027 # object length in meters

# Initial configuration
delta_0: 0.020 #0.0425 # distance from fingertip to object tip within gripper
theta_0: 40

# Intermediate configuration
psi_regrasp: 53.0
theta_tilt: 27
tuck: 3

# Action axis
axis: [1, 0, 0]

# Simulation
sim: 0
'''