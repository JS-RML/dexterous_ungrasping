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
        tuck_angle = config['tuck']
        axis =  config['axis']
        object_thickness = config['object_thickness']
        object_length = config['object_length']
        tcp2fingertip = config['tcp2fingertip']
        sim = config['sim']
        table_height_wrt_world = -0.02

        # If simulation, set robot initial position;
        # Else, read position from real robot. 
        if sim == 1:
            pose = [-0.21, 0.830, 0.01+tcp2fingertip+object_length-delta_0, 0.7071, 0, -0.7071, 0]
            motion_primitives.set_pose(pose)
        else:
            p = group.get_current_pose().pose
            trans_tool0 = [p.position.x, p.position.y, p.position.z]
            rot_tool0 = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] 
            T_wg = tf.TransformerROS().fromTranslationRotation(trans_tool0, rot_tool0)
            P_gg_target = [0, 0, tcp2fingertip+object_length-delta_0, 1]
            P_wg_target = np.matmul(T_wg, P_gg_target)
        
        # Set TCP speed     
        group.set_max_velocity_scaling_factor(tcp_speed)
        
        # Set gripper position
        Robotiq.goto(robotiq_client, pos=object_thickness, speed=config['gripper_speed'], force=config['gripper_force'], block=False)   
        
        center = [pose[0], pose[1], pose[2] - tcp2fingertip - object_length + delta_0]
        visualization.visualizer(center, 1, 0.01, 2)
        
        # Tilt
        tilt.tilt(center, axis, int(90-theta_0), tcp_speed)
        
        # Visualize object during regrasp
        p = group.get_current_pose().pose
        object_v = [center[0]-p.position.x, center[1]-p.position.y, center[2]-p.position.z]
        object_uv = object_v / np.sum(np.power(object_v,2))**0.5
        object_edge = np.multiply(object_uv, object_length)
        visualization.thin_object(center, np.subtract(center, object_edge), object_thickness, 3)
        visualization.visualizer(np.subtract(center, object_edge), 1, 0.01, 4)
        
        # Regrasp
        regrasp.regrasp(np.multiply(axis, -1), int(psi_regrasp), tcp_speed)

        # Tilt
        tilt.tilt(center, axis, int(theta_tilt), tcp_speed)
        
        # Tuck
        tuck.rotate_tuck(np.multiply(axis, -1), int(tuck_angle), 0.03, tcp_speed)

        rospy.spin()
        
    except rospy.ROSInterruptException: pass
