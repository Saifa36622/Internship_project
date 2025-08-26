from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDERECEIVE
import math, time

ROBOT_IP = "192.168.12.60"
rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDERECEIVE(ROBOT_IP)

# (Optional) set your tool center point: [x,y,z,rx,ry,rz] in meters & axis-angle (rad)
# Example: TCP 10 cm in +Z of flange
# rtde_c.setTcp([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])  # set this to YOUR tool, or configure in Installation

def move_to_pose(pose):
    """
    pose: [x, y, z, Rx, Ry, Rz] in base frame, meters + axis-angle (radians).
    """
    qnear = rtde_r.getActualQ()                  # bias solution near current posture
    if not rtde_c.getInverseKinematicsHasSolution(pose, qnear):
        raise RuntimeError(f"IK has no solution for pose: {pose}")
    q = rtde_c.getInverseKinematics(pose, qnear) # compute joints using robot's IK
    # if not rtde_c.isJointsWithinSafetyLimits(q):
    #     raise RuntimeError("IK solution violates safety limits")
    rtde_c.moveJ(q)

# Home in joints (your original)
home_q = [math.radians(45), math.radians(-135), 0.0, math.radians(-45), math.radians(90), math.radians(45)]
rtde_c.moveJ(home_q)

print ("pass")

test_pick = rtde_r.getActualQ()  
print ("Current joint positions:", test_pick)

# + the test pick position
test_pick[0] += 0.1  # Adjust X position for the test pick

print("Test pick position:", test_pick)
# Approach, pick
move_to_pose(test_pick)
# move_to_pose(at_pick)
# input("Press Enter to GRIP...")

# # Retreat
# move_to_pose(above_pick)

# # Place
# move_to_pose(above_place)
# move_to_pose(at_place)
# input("Press Enter to UNGRIP...")

# # Retreat & go home
# move_to_pose(above_place)
# rtde_c.moveJ(home_q, 0.75, 0.75)

rtde_c.stopScript()
print("Script stopped and robot is ready.")
