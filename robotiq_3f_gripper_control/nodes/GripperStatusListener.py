#!/usr/bin/env python3

from __future__ import print_function

import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from std_msgs.msg import UInt8

def gripperStateListener(state):
    state = state.data
    output = "State: " + hex(state) + "-"

    if state == 0xFF:
        output += "INACTICATED"
    if state == 0x80:
        output += "ACTIVATING"
    if state == 0x00:
        output += "ACTIVATIED"
    if state == 0x81:
        output += "PRE_GRASPING"
    if state == 0x01:
        output += "PRE_GRASPED"
    if state == 0x82:
        output += "GRASPING"
    if state == 0x02:
        output += "GRASPED"
    if state == 0x83:
        output += "RELEASING"
    if state == 0x03:
        output += "RELEASED"
    
    print(output)


if __name__ == '__main__':
    rospy.init_node('GripperStateListener')

    rospy.Subscriber('/gripper_state', UInt8, gripperStateListener)

    rospy.spin()
