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

moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)


# TODO: implement rotate tuck
def rotate_tuck(angle_degree, opposite):
    
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    
    msg = rospy.wait_for_message('/CModelRobotInput', inputMsg.CModel_robot_input, timeout = None)
    #Get data of gripper status and calculate width 
    gripper_position = msg.gPO
    gripper_width = ((gripper_position * (-0.6783)) + 147.41)/1000 
    PointB_eelink = [finger_length-opposite, gripper_width/2, 0, 1] 
    PointB_world = numpy.matmul(world2eelink_matrix, PointB_eelink) #Caculate coordinate of point B w.r.t. /world
    print pose_target
    print PointB_world[2], PointB_world[0]
    TurnArcAboutAxis_2('y', PointB_world[2], PointB_world[0], angle_degree, -1, 'yes', 'y', 1)

def tilt(point, axis, magnitude, velocity):
    '''Tilt trajectory of robot. 

    Parameters:
        point (list): 3-D coordinate of point in rotation axis
        axis (list): 3-D vector of rotation axis (right-hand rule)
        magnitude (double): Magnitude of tilt angle
    Returns:
    
    '''
    # Normalize axis vector
    axis = axis/np.linalg.norm(axis)
    
    # Pose variables. The parameters can be seen from "$ rosmsg show Pose"
    pose_target = group.get_current_pose().pose 
    pos_initial = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    ori_initial = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]

    # Tilt center point. Closest point from tcp to axis line    
    center = np.add(point, np.dot(np.subtract(pos_initial, point), axis)*axis)
    
    # Closest distance from tcp to axis line
    radius = np.linalg.norm(np.subtract(center, pos_initial))
    
    # Pair of orthogonal vectors in tilt plane
    v1 =  -np.subtract(np.add(center, np.dot(np.subtract(pos_initial, center), axis)*axis), pos_initial)
    v1 = v1/np.linalg.norm(v1)
    v2 = np.cross(axis, v1)

    # Interpolate orientation poses via quaternion slerp
    q = helper.axis_angle2quaternion(axis, magnitude)
    ori_target =  tf.transformations.quaternion_multiply(q, ori_initial)    
    ori_waypoints = helper.slerp(ori_initial, ori_target, np.arange(1.0/magnitude , 1.1, 1.0/magnitude)) 
    
    waypoints = []
    for t in range(1, magnitude+1):
        circle = np.add(center, radius*(math.cos(math.radians(t)))*v1 + radius*(math.sin(math.radians(t)))*v2)
        pose_target.position.x = circle[0]
        pose_target.position.y = circle[1]
        pose_target.position.z = circle[2]
        pose_target.orientation.x = ori_waypoints[t][0]
        pose_target.orientation.y = ori_waypoints[t][1]
        pose_target.orientation.z = ori_waypoints[t][2]
        pose_target.orientation.w = ori_waypoints[t][3]
        waypoints.append(copy.deepcopy(pose_target))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) # waypoints, resolution=1cm, jump_threshold)
    retimed_plan = group.retime_trajectory(robot.get_current_state(), plan, velocity) # Retime trajectory with scaled velocity
    group.execute(retimed_plan)

if __name__ == '__main__':
    try:
        rospy.init_node('tilt', anonymous=True) #initialize the node 
        
        group.set_max_velocity_scaling_factor(1.0)
        motion_primitives.set_joint([0, -90, 90, 0, 90, 0])  
        #p = group.get_current_pose().pose 
        #tilt([p.position.x+0.1,p.position.x,p.position.x], [0,1,0], 180, 0.5)
    except rospy.ROSInterruptException: pass
