# from rtde_control import RTDEControlInterface
# import math
# rtde_control = RTDEControlInterface("192.168.12.60")

# # joint_q = [math.radians(135), -1.57, 0.0, -1.57, 0.0, 0.0]
# # rtde_control.moveJ(joint_q)
# # joint_q = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0]
# # rtde_control.moveJ(joint_q)
# rtde_control.stopScript()


# tcp control
# -------------------------------------------------------------------------------------------------------------------------

# acceleration = 1.0
# xd = [vx, vy, vz, wx, wy, wz]

# Optional asynchronous=True lets the call return immediately (non-blocking) 

# rtde_control.speedL(xd, acceleration, time=0.0)

# -------------------------------------------------------------------------------------------------------------------------

# moveL_FK(joint_pose, speed, acceleration, asynchronous=False) -> Similar but specifies target by joint angles, planning Cartesian straight-line motion via inverse kinematics .

# -------------------------------------------------------------------------------------------------------------------------

# speedL(xd, acceleration, time=0.0) -> Applies a continuous Cartesian velocity (vector [vx, vy, vz, wx, wy, wz]

# -------------------------------------------------------------------------------------------------------------------------

# speedJ(qd, acceleration, time=0.0)
# Similar to speedL but controls in joint space (joint velocities)

# -------------------------------------------------------------------------------------------------------------------------

# servoL(pose, speed, acceleration, time, lookahead_time, gain)
# Performs real-time Cartesian servoing to a pose.

# Very smooth, good for dynamic updates


# servoJ(...)
# Same idea in joint space

# -------------------------------------------------------------------------------------------------------------------------

# servoC(pose, speed, acceleration, blend)
# Circular continuous servo in Cartesian space with a blend zone .

# movePath(Path, asynchronous=False)
# Defines a path sequence (joint or Cartesian), with speed, acceleration, and blend per waypoint 


# moveP(pose, acceleration, speed, blend)
# Performs circular-then-linear motion like movep in URScript, with blend




# ----------------------------------------------------------------------------
# joint_positions
# [1.6891264915466309, -3.263137479821676, -0.6271414756774902, -3.9827338657774867, -1.567794148121969, 1.7247223854064941]


# tcp
# [0.05275493190660547, 0.293563216956294, -0.23292091859760838, 2.2525255918307456, 2.1669668542228964, -0.013757759548984647]

# ----------------------------------------------------------------------------

from rtde_receive import RTDEReceiveInterface

rtde_receive = RTDEReceiveInterface("192.168.12.60")

# Get current joint positions (in radians)
joint_positions = rtde_receive.getActualQ()
print("Joint positions:", joint_positions)

# Get current end-effector pose (x, y, z, Rx, Ry, Rz)
ee_pose = rtde_receive.getActualTCPPose()
print("End-effector pose:", ee_pose)