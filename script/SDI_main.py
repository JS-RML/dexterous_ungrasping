#!/usr/bin/env python
import sys
import math
import rospy
import copy
import tf
import numpy
import moveit_commander 
import moveit_msgs.msg
import std_msgs.msg 
import geometry_msgs.msg 
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
from apriltags_ros.msg import * 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import *
from rospy import init_node, is_shutdown
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import *

##___GLOBAL VARIABLES___###
velocity = 0.02 #velocity scaling factor (0, 1.0] - Safe value for a real robot is ~0.05
#Dynamixel
goal_pos = float;
goal_speed = 1.0;

##___INITIALIZATION___###
moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #initialize the node 
robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) #publisher that publishes a plan to the topic
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)
regrasp_pub = rospy.Publisher('regrasp_status', String, queue_size = 10)
psi_pub = rospy.Publisher('psi_current', Float32, queue_size = 10)
length_pub = rospy.Publisher('length_value', Float32, queue_size = 10)
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

dynamixel_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)





###___Pick-up Object___###
## This function manipulates gripper and grabs object
## distance is the distance to dive before gripping and velocity is the speed of the motion. It rises 10cm after grabbing object
def pickup(command, distance, vel):
    rospy.sleep(0.5)
    gposition(pub, command, 150) #increment gripper width
    rospy.sleep(1)
    

    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
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
    
    plan_execute_waypoints(waypoints)

    command = outputMsg.CModel_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 20
    command.rFR  = 150						##force need to be adjusted later
    command.rPR = 220
    pub.publish(command)
    rospy.sleep(1)

    
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
   
    x_2 = x_1
    y_2 = y_1
    z_2 = z_1 + 0.1
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
  


#############################################################################################################################################################################################################
####____REGRASP____####
#############################################################################################################################################################################################################

###___REGRASP VERTICAL (VERTICAL CARD REGRASP)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp_Vertical(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2-object_width/4, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            z = PointA_world[2] - math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) - math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            x = PointA_world[0] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))

#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
        #Store Values
        pose_target.position.x = x - object_width*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))+3
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]


###___REGRASP FUNCTION8 (DRY CELL BATTERY REGRASP)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp8(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2845#0.2765 ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x #+0.001*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 153.830405)/(-0.67221463413))
        if position > 187:
            position = position - 2   
        elif position < 187 and position >184:
            position = position + 1
        elif position < 184:
            position = position
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]


###___REGRASP LEGO (LEGO REGRASP)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp_lego(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - 0.001*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z - 0.003*psi_current/psi_target
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))+3
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]



###___REGRASP PICTURE FRAME (PICTURE FRAME REGRASP)___###
## This regrasp function is modified from regrasp7 for the picture frame  
def regrasp_frame(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.283#0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2, 0, 1]
    #PointA_eelink = [finger_length, -object_width/2-object_width/4, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x + 0.0*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z - 0.005*psi_current/psi_target
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]


###___REGRASP FUNCTION7 (CARD REGRASP)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp7(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2-object_width/4, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - object_width*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]







   
def plan_execute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan)

def plan_execute_waypoints_linearpath(waypoints, vel):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, vel) #parameter that changes velocity
    group.execute(plan)
 
def plan_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan, wait = False)

def regrasp_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan, wait = False)

def forceseek_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, 0.002) #parameter that changes velocity
    group.execute(plan, wait = False)


###___MAIN___###
if __name__ == '__main__':

    try:
        
        manipulator_arm_control()
        
        moveit_commander.roscpp_shutdown() #shut down the moveit_commander

    except rospy.ROSInterruptException: pass
