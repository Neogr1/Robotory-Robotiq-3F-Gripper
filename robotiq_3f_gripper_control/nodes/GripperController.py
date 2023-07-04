#!/usr/bin/env python3

"""@package docstring
get the state of Robotiq3FGripper and publish needed status as UINT8
"""

from __future__ import print_function

import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from std_msgs.msg import UInt8
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotInput
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput


#            PRA-SPA-FRA    PRB-SPB-FRB    PRC-SPC-FRC    PRS-SPS-FRS
OBJ_REQ = [((  0,255,150), (  0,255,150), (  0,255,150), (  0,255,150)), # 0. RELEASE
           (( 50,255,150), ( 70,255,150), ( 70,255,150), (128,255,150)), # 1. cleaner
           (( 98,255,150), ( 98,255,150), ( 98,255,150), (150,255,150)), # 2. driver
           ((135,255,150), (138,255,150), (138,255,150), (137,255,150)), # 3. pringles
           (( 70,255,150), ( 70,255,150), ( 70,255,150), (255,255,150)), # 4. banana
           ((255,255,255), (255,255,255), (255,255,255), (137,255,150)), # 5. PET bottle
]

# constants
INACTIVATED  = 0xFF # 1111 1111
ACTIVATING   = 0x80 # 1000 0000
ACTIVATED    = 0x00 # 0000 0000
PRE_GRASPING = 0x81 # 1000 0001
PRE_GRASPED  = 0x01 # 0000 0001
GRASPING     = 0x82 # 1000 0010
GRASPED      = 0x02 # 0000 0010
RELEASING    = 0x83 # 1000 0011
RELEASED     = 0x03 # 0000 0011


current_state = INACTIVATED
command = Robotiq3FGripperRobotOutput()
target_object = 0

def isUnableToRequest():
    ### cannot request while fingers are moving or inactivated
    ### return the most significant bit of current_state
    return (current_state & 0x80) >> 7



# callback state
def updateCurrentState(state):
    global current_state

    if current_state == ACTIVATING:
        if state.gACT == 1 and state.gIMC == 3:
            current_state = ACTIVATED
    
    if current_state == RELEASING:
        if state.gDTA != 0 and state.gDTB != 0 and state.gDTC != 0:
            current_state = RELEASED
    
    if current_state == PRE_GRASPING:
        if state.gDTS != 0:
            current_state = PRE_GRASPED
    
    if current_state == GRASPING:
        if state.gDTA != 0 and state.gDTB != 0 and state.gDTC != 0:
            current_state = GRASPED

    pub_state.publish(UInt8(current_state))


# callback request
def changeCommand(request):
    request = request.data

    global current_state, command, target_object

    if request != ACTIVATED and isUnableToRequest():
        print("The request is ignored since the fingers are moving or the gripper is inactivated.")
        return

    # reset
    if request == INACTIVATED:
        command = Robotiq3FGripperRobotOutput()
        command.rACT = 0
        current_state = INACTIVATED

    # activate
    if request == ACTIVATED:
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
        current_state = ACTIVATING

    # pre-grasp
    if request == PRE_GRASPED:
        command.rPRS, command.rSPS, command.rFRS = OBJ_REQ[target_object][3]
        current_state = PRE_GRASPING
    
    # grasp
    if request == GRASPED:
        command.rPRA, command.rSPA, command.rFRA = OBJ_REQ[target_object][0]
        command.rPRB, command.rSPB, command.rFRB = OBJ_REQ[target_object][1]
        command.rPRC, command.rSPC, command.rFRC = OBJ_REQ[target_object][2]
        current_state = GRASPING

    # release
    if request == RELEASED:
        ### This releasing procedure is not for scissor mode.
        ### If the gripper is on scissor mode, different releasing procedure is needed.
        target_object = 0
        command.rPRA, command.rSPA, command.rFRA = OBJ_REQ[target_object][0]
        command.rPRB, command.rSPB, command.rFRB = OBJ_REQ[target_object][1]
        command.rPRC, command.rSPC, command.rFRC = OBJ_REQ[target_object][2]
        current_state = RELEASING


    pub_request.publish(command)


# callbaek object_id
def setTargetObject(object_id):
    object_id = object_id.data

    global target_object
    target_object = object_id
    print("Target object is set as the number", target_object)



if __name__ == '__main__':
    rospy.init_node('RGripperController')

    # state
    pub_state = rospy.Publisher("/gripper_state", UInt8, queue_size=10)
    rospy.Subscriber("Robotiq3FGripperRobotInput", Robotiq3FGripperRobotInput, updateCurrentState)

    # request
    pub_request = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)
    rospy.Subscriber("/gripper_request", UInt8, changeCommand)

    # object
    rospy.Subscriber("/object_id", UInt8, setTargetObject)

    rospy.spin()
