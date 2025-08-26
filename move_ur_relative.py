from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time
import math
from spatialmath import SE3
# Connect to the robot
rtde_control = RTDEControlInterface("192.168.12.60")
rtde_receive = RTDEReceiveInterface("192.168.12.60")

# Get the current TCP pose
current_pose = rtde_receive.getActualTCPPose()  # [x, y, z, Rx, Ry, Rz]
print("Current TCP Pose:", current_pose)

joint_positions = rtde_receive.getActualQ()
print("Joint positions:", joint_positions)

import numpy as np
import roboticstoolbox as rtb
from math import pi

# Link lengths (a)
a = [0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0]

# Link twists (alpha)
alpha = [pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0]

# Link offsets (d)
d = [0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921]


# Create the list of links
links = []
for i in range(6):
    link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i])
    links.append(link)

# Initialize the UR3e robot model
ur3e = rtb.DHRobot(links, name='UR3e')

ur3e.q = joint_positions

target_pose = current_pose.copy()
target_pose[0] += 0.5  

# Use all 6 values: x, y, z, Rx, Ry, Rz
target_pose_se3 = SE3(target_pose[0], target_pose[1], target_pose[2]) * SE3.RPY(target_pose[3], target_pose[4], target_pose[5])
print("Target TCP Pose:", target_pose_se3)
sol = ur3e.ikine_LM(target_pose_se3, q0=ur3e.q, ilimit=10000, tol=1e-1)



if sol.success:
    joint_positions = sol.q
    # print(f"IK Solution for target {target_pose}: {joint_positions}")

    # Move the robot using the computed joint positions
    print(joint_positions.tolist())


    rtde_control.moveJ(joint_positions.tolist())
else :
    print("IK solution not found for the target pose.")
    print("Solution:", sol)


# Stop RTDE script (if needed)
rtde_control.stopScript()
