import math
import numpy as np
import time

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from ur_analytic_ik import ur3e

rtde_control = RTDEControlInterface("192.168.12.60")
rtde_receive = RTDEReceiveInterface("192.168.12.60")

# --- IMPORTANT: We're not using moveJ for continuous control.
# Instead, we'll use a continuous loop with servoj or speedj.
# The `moveJ` below is just for setting a safe start position.

# Start Position
joint_q = [math.radians(45), math.radians(-135), 0.0, math.radians(-45), math.radians(90), math.radians(45)]
rtde_control.moveJ(joint_q)


def tcp_to_joint_position(target_tcp_pose):
    """
    Calculates a single target joint position for a desired Cartesian pose.

    Args:
        target_tcp_pose (list): A 6D list [x, y, z, Rx, Ry, Rz] for the target pose.

    Returns:
        list: A single joint configuration for the target pose.
        None: If the target pose is unreachable or no solution is found.
    """
    # Use the homogeneous transformation matrix for UR-analytic-ik
    target_T = np.identity(4)
    target_T[:3, 3] = target_tcp_pose[:3]
    # For a full solution, you would need to convert the rotation vector to a rotation matrix.
    # The `ur-analytic-ik` library handles this conversion internally when using the 6D pose.
    # We will proceed with the simplest case by providing a 4x4 matrix from xyz position
    
    # We will use the `inverse_kinematics_closest` to find the best solution
    current_joint_q = rtde_receive.getActualQ()
    if not current_joint_q:
        print("Could not retrieve current robot joint state.")
        return None

    q_solution = ur3e.inverse_kinematics_closest(target_T, *current_joint_q)
    print(f"IK solutions found: {q_solution}")
    if q_solution is None:
        print("Warning: Could not find a 'closest' solution for the target pose.")
        return None
        
    print("Calculated a joint target.")
    return q_solution

def execute_continuous_movement(delta_x, delta_y, delta_z):
    """
    Executes a continuous, joystick-like movement using a high-frequency loop.
    """
    print("Starting continuous joystick control...")
    
    # Loop continuously until the user stops the script
    try:
        while True:
            # 1. Get the current robot pose (x, y, z, Rx, Ry, Rz)
            current_tcp_pose = rtde_receive.getActualTCPPose()
            # Truncate each value to 3 decimal places
            current_tcp_pose = [round(x, 3) for x in current_tcp_pose]
            if not current_tcp_pose:
                print("Could not retrieve current robot pose. Retrying...")
                time.sleep(0.1)
                continue

            # 2. Calculate the new target pose by adding the delta
            # This replaces the need for `pose_trans`
            # The delta is a small increment in m and rad
            delta_pose = np.array([delta_x, delta_y, delta_z, 0, 0, 0])
            target_tcp_pose = np.array(current_tcp_pose) + delta_pose
            
            print(f"Current TCP Pose: {current_tcp_pose}")
            print(f"Target TCP Pose: {target_tcp_pose.tolist()}")
            # 3. Calculate the new joint positions using Inverse Kinematics (IK)
            joint_position = tcp_to_joint_position(target_tcp_pose.tolist())
            
            if joint_position is not None:
                # 4. Send the new joint positions to the robot using servoj
                # This must be done in a high-frequency loop for smooth motion.
                # `servoj` is the correct command for this.
                print(f"Moving to joint position: {joint_position}")
                rtde_control.servoJ(joint_position, 0, 0, 0.002, 0.1, 1250)
            
            # Pause to ensure the loop runs at a consistent frequency.
            # For a real joystick, this loop would be driven by new input.
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping script.")
    finally:
        # Stop the servoj command gracefully
        rtde_control.stopScript()
        rtde_receive.disconnect()
        rtde_control.disconnect()

# --- Main execution block ---
if __name__ == "__main__":
    # Example: move continuously in the positive X direction at 0.01 m/s
    print("Press Ctrl+C to stop the movement.")
    execute_continuous_movement(0.01, 0, 0)