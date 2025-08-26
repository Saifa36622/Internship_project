from rtde_control import RTDEControlInterface
import math
rtde_control = RTDEControlInterface("192.168.12.60")

# start position
joint_q = [math.radians(45), math.radians(-135), 0.0, math.radians(-45), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)

# go right
joint_q = [math.radians(20), math.radians(-150),math.radians(50), math.radians(-140), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)

joint_q = [math.radians(20), math.radians(-185),math.radians(50), math.radians(-140), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)


# grip
input("Press Enter to GRIP (simulate gripper action)...")
# -------------------------

# go left
joint_q = [math.radians(85), math.radians(-150),math.radians(50), math.radians(-140), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)

joint_q = [math.radians(85), math.radians(-170),math.radians(50), math.radians(-150), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)

# ungrip
input("Press Enter to UNGRIP (simulate gripper action)...")
# -------------------------

# home
joint_q = [math.radians(45), math.radians(-135), 0.0, math.radians(-45), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)

# joint_q = [math.radians(20), math.radians(-150),math.radians(50), math.radians(-140), math.radians(90), math.radians(45)]
# rtde_control.moveJ(joint_q)

# joint_q = [math.radians(20), math.radians(-180),math.radians(50), math.radians(-140), math.radians(90), math.radians(45)]
# rtde_control.moveJ(joint_q)


# joint_q = [math.radians(45), math.radians(-135), 0.0, math.radians(-45), math.radians(90), math.radians(45)]
# rtde_control.moveJ(joint_q)

# here
# joint_q = [math.radians(45), math.radians(-180),math.radians(50), math.radians(-140), math.radians(90), math.radians(45)]
# rtde_control.moveJ(joint_q)

# joint_q = [math.radians(45), math.radians(-90), 0.0, math.radians(-90), math.radians(90), math.radians(45)]
# rtde_control.moveJ(joint_q)
# acceleration = 0.25
# xd = [0.5, -0.02830140389876415, 0.6940462639638908, -1.1121563068109643, 1.1133686233561055, 0.0015774743115120119]
# rtde_control.speedL(xd, acceleration)



# joint_q = [math.radians(135), -1.57, 0.0, -1.57, 0.0, 0.0]
# rtde_control.moveJ(joint_q)
# joint_q = [1.6891264915466309, -3.263137479821676, -0.6271414756774902, -3.9827338657774867, -1.567794148121969, 1.7247223854064941]
# rtde_control.moveJ(joint_q)
# joint_q = [math.radians(135), -1.57, 0.0, -1.57, 0.0, 0.0]
# rtde_control.moveJ(joint_q)
rtde_control.stopScript()


# Joint positions: [1.3546977043151855, -3.083353181878561, -1.0211658477783203, -0.3889187139323731, 1.4326720237731934, -0.0016854445086877945]
# End-effector pose: [0.24040898816610337, 0.42621733938067674, -0.07930153709124158, -2.911619060252475, 0.3364414601880378, -0.23655243573538248]

# tcp control
# -------------------------------------------------------------------------------------------------------------------------

# acceleration = 1.0
# xd = [vx, vy, vz, wx, wy, wz]

# Optional asynchronous=True lets the call return immediately (non-blocking) 

# rtde_control.speedL(xd, acceleration, time=0.0)

# -------------------------------------------------------------------------------------------------------------------------

# moveL_FK(joint_pose, speed, acceleration, asynchronous=False) -> Similar but specifies target by joint angles, planning Cartesian straight-line motion via inverse kinematics.

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

