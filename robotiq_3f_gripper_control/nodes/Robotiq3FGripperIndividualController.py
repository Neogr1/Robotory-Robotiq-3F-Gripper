#!/usr/bin/env python


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending commands to a ROS node controlling a 3F gripper gripper.

This serves as an example for publishing messages on the 'Robotiq3FGripperRobotOutput' topic using the 'Robotiq3FGripper_robot_output' msg type for sending commands to a 3F gripper gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

from __future__ import print_function

import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


def genCommand(string, command):
    """Update the command according to the string entered by the user."""
    parsed = string.split(" ")

    if parsed[0] == 'a':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if parsed[0] == 'r':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 0

    if parsed[0] == 'c':
        command.rPRA = 255

    if parsed[0] == 'o':
        command.rPRA = 0

    if parsed[0] == 'b':
        command.rMOD = 0

    if parsed[0] == 'p':
        command.rMOD = 1

    if parsed[0] == 'w':
        command.rMOD = 2

    if parsed[0] == 's':
        command.rMOD = 3

    # If the command entered is a int, assign this value to rPRA
    try:
        command.rPRA = int(parsed[0])
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if parsed[0] == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if parsed[0] == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

    if parsed[0] == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if parsed[0] == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0
    

    ''' added command '''

    if parsed[0] == 'icf0':
        command.rICF = 0

    if parsed[0] == 'icf1':
        # make the values of B and C be the values of A
        command.rPRB = command.rPRA
        command.rSPB = command.rSPA
        command.rFRB = command.rFRA
        command.rPRC = command.rPRA
        command.rSPC = command.rSPA
        command.rFRC = command.rFRA
        command.rICF = 1
    
    if parsed[0] == 'ics0':
        command.rICS = 0

    if parsed[0] == 'ics1':
        command.rSPS = 255
        command.rFRS = 150
        command.rICS = 1

    # control the value of position, speed or force of finger or scissor
    if parsed[0] in ['A', 'B', 'C', 'S']:
        # cannot control fingers individually
        if command.rICF == 0:
            print("rICF must be 1 to control fingers individually.\nInput \"icf1\" first.")
            return command

        if len(parsed) < 3:
            print("Wrong command usage.\nProper example: \"A pos 128\"")

        # get the new value of register from input command
        try:
            value = int(parsed[2])
            if value > 255:
                value = 255
            if value < 0:
                value = 0
        except (ValueError, IndexError):
            return command
        
        if parsed[0] == 'A':
            if parsed[1] == 'pos':
                command.rPRA = value
            if parsed[1] == 'speed':
                command.rSPA = value
            if parsed[1] == 'force':
                command.rFRA = value

        if parsed[0] == 'B':
            if parsed[1] == 'pos':
                command.rPRB = value
            if parsed[1] == 'speed':
                command.rSPB = value
            if parsed[1] == 'force':
                command.rFRB = value

        if parsed[0] == 'C':
            if parsed[1] == 'pos':
                command.rPRC = value
            if parsed[1] == 'speed':
                command.rSPC = value
            if parsed[1] == 'force':
                command.rFRC = value

        if parsed[0] == 'S':
            if parsed[1] == 'pos':
                command.rPRS = value
            if parsed[1] == 'speed':
                command.rSPS = value
            if parsed[1] == 'force':
                command.rFRS = value
        
    

    return command

def makeCurrentCommand(command):
    currentCommand = '\n==============================\n'
    currentCommand += 'Individual 3F Gripper Controller\n'
    currentCommand += 'Current command:\n'
    currentCommand += 'Activated : ' + str(command.rACT) + '\n'
    currentCommand += 'Mode      : ' + str(command.rMOD) + '\n'
    currentCommand += 'Individual: ' + str(command.rICF) + '\n'
    currentCommand += 'Scissor   : ' + str(command.rICS) + '\n'
    currentCommand += '\n'
    currentCommand += '         | pos | speed | force\n'
    currentCommand += '------------------------------\n'
    currentCommand += 'finger A | %3d |   %3d |   %3d\n' %(command.rPRA, command.rSPA, command.rFRA)
    currentCommand += 'finger B | %3d |   %3d |   %3d\n' %(command.rPRB, command.rSPB, command.rFRB)
    currentCommand += 'finger C | %3d |   %3d |   %3d\n' %(command.rPRC, command.rSPC, command.rFRC)
    currentCommand += 'scissor  | %3d |   %3d |   %3d\n' %(command.rPRS, command.rSPS, command.rFRS)
    currentCommand += '------------------------------\n'

    return currentCommand


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand = makeCurrentCommand(command)
    print(currentCommand)
    
    strAskForCommand = 'Available commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    strAskForCommand += '\n'
    strAskForCommand += 'icf0: Disable individual control of fingers\n'
    strAskForCommand += 'icf1: Enable individual control of fingers\n'
    strAskForCommand += 'ics0: Disable individual control of scissor axis\n'
    strAskForCommand += 'ics1: Enable individual control of scissor axis\n'
    strAskForCommand += '(A,B,C,S) (pos,speed,force) (0-255)\n'
    strAskForCommand += '    : Individual control of fingers or scissor axis\n'
    strAskForCommand += '-->'

    return raw_input(strAskForCommand)


def publisher():
    """Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic."""

    rospy.init_node('Robotiq3FGripperSimpleController')

    pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)

    command = Robotiq3FGripperRobotOutput();

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)

        pub.publish(command)

        rospy.sleep(0.1)


if __name__ == '__main__':
    publisher()
