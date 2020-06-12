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
import dynamixel

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
        tuck_angle = config['tuck']
        axis =  config['axis']
        object_thickness = config['object_thickness']
        object_length = config['object_length']
        tcp2fingertip = config['tcp2fingertip']
        sim = config['sim']
        table_height_wrt_world = -0.02

        # Set TCP speed     
        group.set_max_velocity_scaling_factor(tcp_speed)
        
        # POSES
        prepick_pose=[0.6182, -0.2323, 0.3579, 0, 0.7071, 0, 0.7071]
        prepick_joint=[3.0332820415496826, -1.620906178151266, -2.052493397389547, -1.0383504072772425, 1.5700675249099731, -0.10923844972719365]
        pick_pose=[0.6184, -0.2327, 0.289, 0, 0.7071, 0, 0.7071]
        precontact_joint=[2.431755542755127, -2.057488266621725, -1.3988855520831507, -1.837355915700094, 2.135869026184082, -0.8838380018817347]
        initial_contact_joint=[2.431455373764038, -2.083695713673727, -1.445674244557516, -1.7674606482135218, 2.144381523132324, -0.8864739576922815]
        detach_joint=[2.6432394981384277, -1.730476204548971, -1.6622641722308558, -1.3673556486712855, 1.6674232482910156, -0.5001123587237757]

        
        # Iniitalize gripper
        dynamixel.set_length(115)
        Robotiq.goto(robotiq_client, pos=object_thickness+0.02, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        rospy.sleep(0.5)
        
        # Picking routine
        motion_primitives.set_joint_radians(prepick_joint)
        rospy.sleep(0.5)
        Robotiq.goto(robotiq_client, pos=object_thickness+0.02, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        rospy.sleep(0.5)
        motion_primitives.set_pose(pick_pose)
        rospy.sleep(1)
        Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        rospy.sleep(1)
        motion_primitives.set_joint_radians(prepick_joint)
        rospy.sleep(0.5)
        
        # Making initial contact
        motion_primitives.set_joint_radians(precontact_joint)
        rospy.sleep(0.5)
        motion_primitives.set_joint_radians(initial_contact_joint)
             
        # read position from real robot. 
        p = group.get_current_pose().pose
        trans_tool0 = [p.position.x, p.position.y, p.position.z]
        rot_tool0 = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] 
        T_wg = tf.TransformerROS().fromTranslationRotation(trans_tool0, rot_tool0)
        P_g_center = [tcp2fingertip+object_length-delta_0, object_thickness/2, 0, 1]
        P_w_center = np.matmul(T_wg, P_g_center)
        center = P_w_center[:3]

        # Regrasp
        regrasp.palm_regrasp(np.multiply(axis, -1), int(psi_regrasp), tcp_speed)
        rospy.sleep(0.5)
        
        # Tilt
        tilt.tilt(center, axis, int(theta_tilt), tcp_speed)
        rospy.sleep(0.5)
        
        # Surface Slide
        motion_primitives.set_pose_relative([0, 0.008, 0])
        rospy.sleep(0.5)
        
        # Push-Tuck        
        tuck.push_tuck(np.multiply(axis, -1), int(tuck_angle), 0.01, tcp_speed, 130)
        rospy.sleep(0.5)
        
        # Detach
        motion_primitives.set_pose_relative([0, -0.005, 0])
        rospy.sleep(0.5)
        motion_primitives.set_joint_radians(detach_joint)
        
        
        p =group.get_current_joint_values()
        print p
        #rospy.spin()
                
    except rospy.ROSInterruptException: pass
        
'''
# Robot parameters 
tcp_speed: 0.05

# Gripper parameters
tcp2fingertip: 0.28 #0.275 # distance from tcp to gripper fingertip
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
psi_regrasp: 54.0
theta_tilt: 22
tuck: 7

# Action axis
axis: [1, 0, 0]

# Simulation
sim: 0
'''
