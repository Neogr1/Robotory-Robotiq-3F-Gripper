import time
import math
import pybullet as p
import pybullet_data

client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81, physicsClientId=client) 
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
gripperId = p.loadURDF("robotiq-3f-gripper_articulated.urdf", basePosition=[0,0,0.05], baseOrientation=[1,0,0,1], useFixedBase=True)

# joint_index = -1  # Replace with the actual index of the joint you want to adjust

num_joints = p.getNumJoints(gripperId)

'''
[joints]
 0 palm_finger_1_joint
 1 finger_1_joint_1
 2 finger_1_joint_2
 3 finger_1_joint_3
 4 palm_finger_2_joint
 5 finger_2_joint_1
 6 finger_2_joint_2
 7 finger_2_joint_3
 8 palm_finger_middle_joint
 9 finger_middle_joint_1
10 finger_middle_joint_2
11 finger_middle_joint_3
'''

COEF_JOINT_1 = 62 / 140
COEF_JOINT_2 = 90 / 100

rPOA_slider = p.addUserDebugParameter("rPOA", 0, 255, 0)
rPOB_slider = p.addUserDebugParameter("rPOB", 0, 255, 0)
rPOC_slider = p.addUserDebugParameter("rPOC", 0, 255, 0)
rPOS_slider = p.addUserDebugParameter("rPOS", 0, 255, 137)
rICF_slider = p.addUserDebugParameter("rICF", 0, 1, 0)

p.addUserDebugText("test",[0.1,0.1,0.1], textColorRGB=[0,0,0], textSize=10)

while 1:
    rPOA = p.readUserDebugParameter(rPOA_slider)
    rPOB = p.readUserDebugParameter(rPOB_slider)
    rPOC = p.readUserDebugParameter(rPOC_slider)
    rPOS = p.readUserDebugParameter(rPOS_slider)
    rICF = p.readUserDebugParameter(rICF_slider)

    f1_0 = min(rPOS, 220) / 220 * 26 - 16
    f2_0 = -f1_0
    fm_0 = 0

    fm_1 = COEF_JOINT_1 * min(rPOA, 140) + 8
    fm_2 = COEF_JOINT_2 * min(max(rPOA-140, 0), 100)
    fm_3 = - COEF_JOINT_1 * min(rPOA, 110)

    if rICF < 0.5:
        f1_1, f1_2, f1_3 = fm_1, fm_2, fm_3
        f2_1, f2_2, f2_3 = fm_1, fm_2, fm_3
    else:
        f1_1 = COEF_JOINT_1 * min(rPOC, 140) + 8
        f1_2 = COEF_JOINT_2 * min(max(rPOC-140, 0), 100)
        f1_3 = - COEF_JOINT_1 * min(rPOC, 110)
        f2_1 = COEF_JOINT_1 * min(rPOB, 140) + 8
        f2_2 = COEF_JOINT_2 * min(max(rPOB-140, 0), 100)
        f2_3 = - COEF_JOINT_1 * min(rPOB, 110)
    

    p.setJointMotorControl2(gripperId,  0, p.POSITION_CONTROL, targetPosition=math.radians(f1_0))
    p.setJointMotorControl2(gripperId,  1, p.POSITION_CONTROL, targetPosition=math.radians(f1_1))
    p.setJointMotorControl2(gripperId,  2, p.POSITION_CONTROL, targetPosition=math.radians(f1_2))
    p.setJointMotorControl2(gripperId,  3, p.POSITION_CONTROL, targetPosition=math.radians(f1_3))
    p.setJointMotorControl2(gripperId,  4, p.POSITION_CONTROL, targetPosition=math.radians(f2_0))
    p.setJointMotorControl2(gripperId,  5, p.POSITION_CONTROL, targetPosition=math.radians(f2_1))
    p.setJointMotorControl2(gripperId,  6, p.POSITION_CONTROL, targetPosition=math.radians(f2_2))
    p.setJointMotorControl2(gripperId,  7, p.POSITION_CONTROL, targetPosition=math.radians(f2_3))
    p.setJointMotorControl2(gripperId,  8, p.POSITION_CONTROL, targetPosition=math.radians(fm_0))
    p.setJointMotorControl2(gripperId,  9, p.POSITION_CONTROL, targetPosition=math.radians(fm_1))
    p.setJointMotorControl2(gripperId, 10, p.POSITION_CONTROL, targetPosition=math.radians(fm_2))
    p.setJointMotorControl2(gripperId, 11, p.POSITION_CONTROL, targetPosition=math.radians(fm_3))

    p.stepSimulation()
    time.sleep(0.1)