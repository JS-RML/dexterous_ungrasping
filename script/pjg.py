#!/usr/bin/env python

'''
GRIPPER OUTPUT REGISTERS & FUNCTIONALITES:

rACT (ACTION REQUEST): First action to be made prior to any other actions, rACT bit will activate the Grpiper. Clear rACT to reset the Gripper and clear fault status.

rGTO: The "Go To" action moves the Gripper fingers to the requested position using the configuration defined by the other registers,
rGTO will engage motion while byte 3, 4 and 5 will determine aimed position, force and speed. The only motions performed without
the rGTO bit are activation and automatic release routines

rATR: Automatic Release routine action slowly opens the Gripper fingers until all motion axes reach their mechanical limits. After all
motion is completed, the Gripper sends a fault signal and needs to be reactivated before any other motion is performed. The rATR bit
overrides all other commands excluding the activation bit (rACT).

rPR (POSITION REQUEST): This register is used to set the target position for the Gripper's fingers. The positions 0x00 and 0xFF correspond respectively to the fully
opened and fully closed mechanical stops. For detailed finger trajectory, please refer to the Specifications section.
- 0x00 - Open position, with 140 mm opening respectively
- 0xFF - Closed
- Opening / count: 0.65 mm (for 140 mm stroke)
(INFO: The activation feature of the Robotiq Adaptive Gripper will allow the Gripper to adjust to any fingertips. No matter what is
the size and shape of the fingertips used, 0 will always be fully opened and 255 fully closed, with a quasi-linear relationship
between 0 and 255.)

rSP (SPEED): This register is used to set the Gripper closing or opening speed in real time, however, setting a speed will not initiate a motion.
- 0x00 - Minimum speed
- 0xFF - Maximum speed

rFR (FORCE): The force setting defines the final gripping force for the Gripper. The force will fix the maximum current sent to the motor while in
motion. If the current limit is exceeded, the fingers stop and trigger an object detection notification. Please refer to the Robot Input
Registers & Status section for details on force control.
- 0x00 - Minimum force
- 0xFF - Maximum force

GRIPPER INPUT REGISTERS & STATUS:

gACT: Activation status, echo of the rACT bit (activation bit).

gGTO: Action status, echo of the rGTO bit (go to bit).

gSTA: Gripper status, returns the current status & motion of the Gripper fingers.
- 0x00 - Gripper is in reset ( or automatic release ) state. See Fault Status if Gripper is activated.
- 0x01 - Activation in progress.
- 0x02 - Not used.
- 0x03 - Activation is completed.

gOBJ: Object detection status, is a built-in feature that provides information on possible object pick-up. Ignore if gGTO == 0.
- 0x00 - Fingers are in motion towards requested position. No object detected.
- 0x01 - Fingers have stopped due to a contact while opening before requested position. Object detected opening.
- 0x02 - Fingers have stopped due to a contact while closing before requested position. Object detected closing.
- 0x03 - Fingers are at requested position. No object detected or object has been loss / dropped.

gFLT: Fault status returns general error messages that are useful for troubleshooting. Fault LED (red) is present on the Gripper chassis,
LED can be blue, red or both and be solid or blinking.
- 0x00 - No fault (LED is blue)
- Priority faults (LED is blue)
    0x05 - Action delayed, activation (reactivation) must be completed prior to perfmoring the action.
    0x07 - The activation bit must be set prior to action.
- Minor faults (LED continuous red)
    0x08 - Maximum operating temperature exceeded, wait for cool-down.
    0x09 No communication during at least 1 second.
- Major faults (LED blinking red/blue) - Reset is required (rising edge on activation bit rACT needed).
    0x0A - Under minimum operating voltage.
    0x0B - Automatic release in progress.
    0x0C - Internal fault; contact support@robotiq.com.
    0x0D - Activation fault, verify that no interference or other error occurred.
    0x0E - Overcurrent triggered.
    0x0F - Automatic release completed.

gPR: Echo of the requested position for the Gripper, value between 0x00 and 0xFF.
gPO: Actual position of the Gripper obtained via the encoders, value between 0x00 and 0xFF.
gCU: The current is read instantaneously from the motor drive, value between 0x00 and 0xFF, approximate current equivalent is 10 * value read in mA.
'''

import sys
import rospy 
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg

rospy.init_node('pjg', anonymous=True)  
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)

def activate():
    command = outputMsg.CModel_robot_output()
    command.rACT = 1
    pub.publish(command)
    rospy.sleep(0.5)
    command.rGTO = 1
    command.rPr  = 255
    command.rSP  = 50
    command.rFR  = 150			
    pub.publish(command)
    rospy.sleep(0.5)
  
def reset():
    command = outputMsg.CModel_robot_output()
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.5)

def set_position(position):   
    command = outputMsg.CModel_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 50
    command.rFR  = 150						
    command.rPR = position
    pub.publish(command)
    rospy.sleep(0.5)

# TODO: def set_speed
# TODO: def set_force
# TODO: def automatic_release

if __name__ == '__main__':
    try:
        print "pjg.py"
    except rospy.ROSInterruptException: pass
