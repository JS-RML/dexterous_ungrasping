#!/usr/bin/env python
import sys
import rospy
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import *

rospy.init_node('dynamixel', anonymous=True)  
dynamixel_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

#Dynamixel
goal_pos = float;
goal_speed = 1.0;

###___Call set_speed service to set speed of motor (Range: 0~255)___###
def increment_speed(increment):
    global goal_speed

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 
        if increment == 1:
            setspeed(goal_speed + 0.2)
            goal_speed = goal_speed + 0.2
            print (goal_speed)
        if increment == -1:
            setspeed(goal_speed - 0.2)
            goal_speed = goal_speed - 0.2
            print (goal_speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

###___Set speed of the linear actuator from a range of 0 to 2___###
def set_speed(speed):

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 
        setspeed(speed)
        print (speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e  


###___Set position of the linear actuator where 0 is fully contracted and 255 is fully extended___###
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
    dynamixel_pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))

###___Set position of the linear actuator where 0 is fully contracted and 255 is fully extended___###
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
    dynamixel_pub.publish(Float64(goal_pos))
    rospy.loginfo('goal_pos: {0}' .format(goal_pos))

def set_length(L):
    if L <= 152 and L >= 108:
        angle = (-5.757436269715072e-13)*L**8+(4.086021215764263e-10)*L**7-(1.235505184914909e-07)*L**6+(2.073314838537072e-05)*L**5-(0.002105378201679)*L**4+(0.131937131828588)*L**3-(4.951404238930521)*L**2+(99.851422343330500)*L-(6.984240028045563e+02);
        set_angle(angle)

if __name__ == '__main__':
    try:
        print "dynamixel.py"
    except rospy.ROSInterruptException: pass
