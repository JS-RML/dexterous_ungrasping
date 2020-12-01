#! /usr/bin/env python
"Tesing with the gripper with position and force input"

import rospy

# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

def operate_gripper():

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    robotiq_client.wait_for_server()
    
    pos_value = 0.1
    speed_value = 0.1
    force_value = 10

    while not rospy.is_shutdown():
        command = str(raw_input("Enter command:")).lower()
        action = command[0]
        value = float(command[1:])
        if action == 'p':
            pos_value = value
        elif action == 's':
            speed_value = value
        elif action == 'f':
            force_value = value

        Robotiq.goto(robotiq_client, pos=pos_value, speed=speed_value, force=force_value , block=False)

    
    rospy.spin()


if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('robotiq_2f_client')
    operate_gripper()