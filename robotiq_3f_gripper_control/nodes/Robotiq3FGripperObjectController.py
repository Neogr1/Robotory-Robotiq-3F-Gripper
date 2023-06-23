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

# (rPRA, rPRB, rPRC, rPRS)
POS_FOR_OBJECT = (( 50, 70, 70,128),    # 0 - Cleaner
                  ( 98, 98, 98,150),    # 1 - Cross driver
                  (135,138,138,137),    # 2 - Pringles
                  ( 70, 70, 70,255),    # 3 - Bananas
                  ( 73, 73, 73,130),    # 4 - Georgia (vertical)
)

def copyCommand(command):
    newCommand = Robotiq3FGripperRobotOutput();
    newCommand.rACT = command.rACT
    newCommand.rMOD = command.rMOD
    newCommand.rGTO = command.rGTO
    newCommand.rATR = command.rATR
    newCommand.rGLV = command.rGLV
    newCommand.rICF = command.rICF
    newCommand.rICS = command.rICS
    newCommand.rPRA = command.rPRA
    newCommand.rSPA = command.rSPA
    newCommand.rFRA = command.rFRA
    newCommand.rPRB = command.rPRB
    newCommand.rSPB = command.rSPB
    newCommand.rFRB = command.rFRB
    newCommand.rPRC = command.rPRC
    newCommand.rSPC = command.rSPC
    newCommand.rFRC = command.rFRC
    newCommand.rPRS = command.rPRS
    newCommand.rSPS = command.rSPS
    newCommand.rFRS = command.rFRS

    return newCommand

def genCommandList(string, command):
    """Update the command according to the string entered by the user."""
    parsed = string.split(" ")
    commandList = []

    if parsed[0] == 'a':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rICF = 1
        command.rICS = 1

        command.rSPA = 255
        command.rFRA = 150

        command.rSPB = 255
        command.rFRB = 150

        command.rSPC = 255
        command.rFRC = 150

        command.rPRS = 137
        command.rSPS = 255
        command.rFRS = 150

        commandList.append(command)

    if parsed[0] == 'r':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 0

        commandList.append(command)

    if parsed[0] == 'open':
        command.rPRA, command.rPRB, command.rPRC = (0,0,0)

        commandList.append(command)

    if parsed[0] == 'grip':
        # get the target object number
        try:
            object = int(parsed[1])
        except (ValueError, IndexError):
            print("WRONG COMMAND USAGE.\nProper example: \"grip 0\"")
            return command
        
        # get the finger positions for target object
        try:
            positions = POS_FOR_OBJECT[object]
        except IndexError:
            print("No such object number")
            return command
        
        # scissor axis moves first
        command.rPRS = positions[3]
        commandList.append(command)

        # fingers move later
        newCommand = copyCommand(command)
        newCommand.rPRA, newCommand.rPRB, newCommand.rPRC = positions[:3]
        commandList.append(newCommand)

    return commandList

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
    strAskForCommand += '\n'
    strAskForCommand += 'open      : open fingers (release object)\n'
    strAskForCommand += 'grip (0-4): grip target object\n'
    strAskForCommand += '\t0 - Cleaner\n'
    strAskForCommand += '\t1 - Cross driver\n'
    strAskForCommand += '\t2 - Pringles\n'
    strAskForCommand += '\t3 - Bananas\n'
    strAskForCommand += '\t4 - Georgia (vertical)\n'
    strAskForCommand += '-->'

    return raw_input(strAskForCommand)


def publisher():
    """Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic."""

    rospy.init_node('Robotiq3FGripperObjectController')

    pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)

    command = Robotiq3FGripperRobotOutput();

    while not rospy.is_shutdown():
        inputCommand = askForCommand(command)

        # move scissor axis first
        commandList = genCommandList(inputCommand, command)

        if len(commandList) < 1:
            continue

        command = commandList[0]
        pub.publish(command)

        if len(commandList) == 2:
            # wait for scissor axis to stop moving
            # need to be change to while loop that checks the movements stops with status listener (subscribe something)
            # or caclulate moving time with the difference of original position and target position
            rospy.sleep(3)

            command = commandList[1]
            pub.publish(command)

        rospy.sleep(0.1)


if __name__ == '__main__':
    publisher()
