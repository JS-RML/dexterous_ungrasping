#!/usr/bin/env python
import sys
import math
import rospy
from rospy import init_node, is_shutdown
from std_msgs.msg import Float64
from std_msgs.msg import String
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import *

##___Global Variables___###
goal_pos = float;
goal_speed = 1.0;
pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 

# Increment speed of dynamixel (increment = -1 or 1)
def increment_speed(increment):
    global goal_speed
    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        if increment == 1:
            setspeed(goal_speed + 0.2)
            goal_speed = goal_speed + 0.2
            print "Dynamixel speed is increased to: ", goal_speed
        if increment == -1:
            setspeed(goal_speed - 0.2)
            goal_speed = goal_speed - 0.2
            print "Dynamixel speed is decreased to: ", goal_speed
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

# Set speed of the linear actuator from a range of 0 to 2
def set_speed(speed):
    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed(speed)
        print "Dynamixel speed is set to: ", speed
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e  

# Set position of the linear actuator where 0 is fully contracted and 255 is fully extended
def set_position(position):
    global goal_pos
    
    extend_max = -2;
    contract_max = 1.5;
    goal_pos = position
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))
    goal_pos = 1.5-(goal_pos/73.0)
	
    if goal_pos < extend_max:
        goal_pos = extend_max
    if goal_pos > contract_max:
        goal_pos = contract_max
    pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))


# Set angle of the proximal link of actuator from 0 to 90
def set_angle(position):
    global goal_pos
    
    extend_max = -2;
    contract_max = 1.5;
    goal_pos = position
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))
    #goal_pos = 1.5-(goal_pos/73.0)
    goal_pos = 0.031889*goal_pos-1.65
	
    if goal_pos < extend_max:
        goal_pos = extend_max
    if goal_pos > contract_max:
        goal_pos = contract_max
    pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))

# Set extension length of linear actuator from a range of 45 to 152
def set_length(L):
    angle = (-5.757436269715072e-13)*L**8+(4.086021215764263e-10)*L**7-(1.235505184914909e-07)*L**6+(2.073314838537072e-05)*L**5-(0.002105378201679)*L**4+(0.131937131828588)*L**3-(4.951404238930521)*L**2+(99.851422343330500)*L-(6.984240028045563e+02);
    if angle <= 101 and angle >= -15:
        set_angle(angle)


###___Get command from user to control the gripper (e > extend, c > contract, f > faster, s > slower, 0~255 position)___###
def get_Key(data):
    user_command = raw_input()   
    if user_command in ['e']:
        set_position(255)
        print("Extend")
    elif user_command in ['c']:
        set_position(0)
        print("Contract")
    elif user_command in ['f']:
        increment_speed(1)
        print("Faster")
    elif user_command in ['s']:
        increment_speed(-1)
        print("Slower")
    elif user_command in ['x']:
        set_speed(0)
        print("Emergency Stop")
    else:
        set_pos = int(user_command)
        if set_pos <= 152 and set_pos >= 108:
            set_length(set_pos)  
     
###___Initiate node; subscribe to topic; call callback function___###
def dynamixel_control():
    rospy.init_node('dynamixel_control', anonymous=True)
    rospy.Subscriber('/tilt_controller/state', JointState, get_Key)
    rospy.spin()

if __name__ == '__main__':
    try:
        dynamixel_control()
    except rospy.ROSInterruptException: pass


