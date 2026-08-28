import sys
import os

fairino_path = os.path.join(os.path.dirname(__file__), '..', 'fairino')
sys.path.insert(0, fairino_path)

import Robot


robot = Robot.RPC('192.168.58.2')
n=0
while(n<20):
    print("\n\nn:",n)
    error, joint = robot.GetActualJointPosDegree()
    # 这是一个角度
    print("GetActualJointPosDegree return:", error,joint)

    joint[0]=joint[0]+20
    joint[2]=joint[2]+20
    robot.MoveJ(joint,0,0)
    print("MoveJ return:", error)

    error = robot.GetJointDriverTorque()
    print("GetJointDriverTorque return:", error)

    error = robot.GetJointDriverTemperature()
    print("GetJointDriverTemperature return:", error)

    joint[0]=joint[0]-20
    joint[2]=joint[2]-20
    robot.MoveJ(joint,0,0)
    print("MoveJ return:", error)
    n=n+1